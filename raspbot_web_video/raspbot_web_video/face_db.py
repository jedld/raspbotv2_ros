"""
SQLite-backed face embedding database for persistent face recognition.

Stores 128-d SFace embeddings with associated face crops.  Supports
matching by cosine similarity, merging identities, and CRUD operations
exposed through a simple Python API.

Thread / multi-process safe via WAL mode.
"""

from __future__ import annotations

import io
import os
import sqlite3
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
_DEFAULT_DB_DIR = os.path.expanduser("~/.local/share/raspbot")
_DEFAULT_DB_NAME = "face_db.sqlite"

EMBEDDING_DIM = 128  # SFace output


# ---------------------------------------------------------------------------
# Helper – cosine similarity for a pair of L2-normalised vectors
# ---------------------------------------------------------------------------
def _cosine_sim(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.dot(a, b))


# ---------------------------------------------------------------------------
# Database class
# ---------------------------------------------------------------------------
class FaceDB:
    """Persistent face embedding store backed by SQLite."""

    def __init__(self, db_path: str | None = None):
        if db_path is None:
            db_dir = _DEFAULT_DB_DIR
            os.makedirs(db_dir, exist_ok=True)
            db_path = os.path.join(db_dir, _DEFAULT_DB_NAME)
        self._db_path = db_path
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA foreign_keys=ON")
        self._create_tables()
        # In-memory centroid cache: face_id -> normalised centroid ndarray
        self._centroids: dict[int, np.ndarray] = {}
        self._rebuild_centroids()

    # ------------------------------------------------------------------
    # Schema
    # ------------------------------------------------------------------
    def _create_tables(self) -> None:
        self._conn.executescript("""
            CREATE TABLE IF NOT EXISTS faces (
                face_id     INTEGER PRIMARY KEY AUTOINCREMENT,
                name        TEXT    NOT NULL DEFAULT '',
                created_at  REAL    NOT NULL
            );
            CREATE TABLE IF NOT EXISTS embeddings (
                id          INTEGER PRIMARY KEY AUTOINCREMENT,
                face_id     INTEGER NOT NULL REFERENCES faces(face_id) ON DELETE CASCADE,
                embedding   BLOB    NOT NULL,
                crop_jpeg   BLOB,
                created_at  REAL    NOT NULL
            );
            CREATE INDEX IF NOT EXISTS idx_emb_face ON embeddings(face_id);
        """)
        self._conn.commit()

    # ------------------------------------------------------------------
    # Centroid cache helpers
    # ------------------------------------------------------------------
    def _rebuild_centroids(self) -> None:
        """Rebuild the in-memory centroid cache from all stored embeddings."""
        self._centroids.clear()
        cur = self._conn.execute("SELECT face_id FROM faces")
        for (fid,) in cur.fetchall():
            self._update_centroid(fid)

    def _update_centroid(self, face_id: int) -> None:
        """Re-compute centroid for a single face from its stored embeddings."""
        cur = self._conn.execute(
            "SELECT embedding FROM embeddings WHERE face_id = ?", (face_id,)
        )
        rows = cur.fetchall()
        if not rows:
            self._centroids.pop(face_id, None)
            return
        arrs = [np.frombuffer(r[0], dtype=np.float32).copy() for r in rows]
        centroid = np.mean(arrs, axis=0)
        norm = np.linalg.norm(centroid)
        if norm > 0:
            centroid /= norm
        self._centroids[face_id] = centroid

    # ------------------------------------------------------------------
    # CRUD – faces
    # ------------------------------------------------------------------
    def add_face(self, name: str = "") -> int:
        cur = self._conn.execute(
            "INSERT INTO faces (name, created_at) VALUES (?, ?)",
            (name, time.time()),
        )
        self._conn.commit()
        return cur.lastrowid  # type: ignore[return-value]

    def get_face(self, face_id: int) -> Optional[dict]:
        cur = self._conn.execute(
            "SELECT face_id, name, created_at FROM faces WHERE face_id = ?",
            (face_id,),
        )
        row = cur.fetchone()
        if row is None:
            return None
        n_emb = self._conn.execute(
            "SELECT COUNT(*) FROM embeddings WHERE face_id = ?", (face_id,)
        ).fetchone()[0]
        return {
            "face_id": row[0],
            "name": row[1],
            "created_at": row[2],
            "num_embeddings": n_emb,
        }

    def get_all_faces(self) -> List[dict]:
        cur = self._conn.execute(
            "SELECT f.face_id, f.name, f.created_at, COUNT(e.id) "
            "FROM faces f LEFT JOIN embeddings e ON f.face_id = e.face_id "
            "GROUP BY f.face_id ORDER BY f.face_id"
        )
        return [
            {"face_id": r[0], "name": r[1], "created_at": r[2], "num_embeddings": r[3]}
            for r in cur.fetchall()
        ]

    def update_face_name(self, face_id: int, new_name: str) -> bool:
        cur = self._conn.execute(
            "UPDATE faces SET name = ? WHERE face_id = ?", (new_name, face_id)
        )
        self._conn.commit()
        return cur.rowcount > 0

    def delete_face(self, face_id: int) -> bool:
        cur = self._conn.execute(
            "DELETE FROM faces WHERE face_id = ?", (face_id,)
        )
        self._conn.commit()
        self._centroids.pop(face_id, None)
        return cur.rowcount > 0

    def clear_all(self) -> int:
        n = self._conn.execute("SELECT COUNT(*) FROM faces").fetchone()[0]
        self._conn.execute("DELETE FROM embeddings")
        self._conn.execute("DELETE FROM faces")
        self._conn.commit()
        self._centroids.clear()
        return n

    def merge_faces(self, keep_id: int, merge_id: int) -> bool:
        """Move all embeddings from *merge_id* into *keep_id*, then delete *merge_id*."""
        if keep_id == merge_id:
            return False
        if not self.get_face(keep_id) or not self.get_face(merge_id):
            return False
        self._conn.execute(
            "UPDATE embeddings SET face_id = ? WHERE face_id = ?",
            (keep_id, merge_id),
        )
        self._conn.execute("DELETE FROM faces WHERE face_id = ?", (merge_id,))
        self._conn.commit()
        self._update_centroid(keep_id)
        self._centroids.pop(merge_id, None)
        return True

    # ------------------------------------------------------------------
    # CRUD – embeddings
    # ------------------------------------------------------------------
    def add_embedding(
        self, face_id: int, embedding: np.ndarray, crop_jpeg: bytes | None = None
    ) -> int:
        blob = embedding.astype(np.float32).tobytes()
        cur = self._conn.execute(
            "INSERT INTO embeddings (face_id, embedding, crop_jpeg, created_at) VALUES (?, ?, ?, ?)",
            (face_id, blob, crop_jpeg, time.time()),
        )
        self._conn.commit()
        self._update_centroid(face_id)
        return cur.lastrowid  # type: ignore[return-value]

    def get_face_embeddings(self, face_id: int) -> List[dict]:
        cur = self._conn.execute(
            "SELECT id, created_at FROM embeddings WHERE face_id = ? ORDER BY id",
            (face_id,),
        )
        return [{"id": r[0], "created_at": r[1]} for r in cur.fetchall()]

    def get_thumbnail(self, face_id: int) -> Optional[bytes]:
        """Return the most recent crop JPEG for *face_id*, or None."""
        cur = self._conn.execute(
            "SELECT crop_jpeg FROM embeddings WHERE face_id = ? AND crop_jpeg IS NOT NULL "
            "ORDER BY id DESC LIMIT 1",
            (face_id,),
        )
        row = cur.fetchone()
        return row[0] if row else None

    # ------------------------------------------------------------------
    # Matching
    # ------------------------------------------------------------------
    def match(
        self,
        embedding: np.ndarray,
        threshold: float = 0.363,
    ) -> Optional[Tuple[int, str, float]]:
        """
        Find the best matching face for the given embedding.

        Returns (face_id, name, similarity) if above *threshold*, else None.
        The SFace paper recommends 0.363 for cosine distance on the 2021 model.
        """
        if not self._centroids:
            return None

        emb = embedding.astype(np.float32)
        norm = np.linalg.norm(emb)
        if norm > 0:
            emb = emb / norm

        best_id = -1
        best_sim = -1.0
        best_name = ""
        for fid, centroid in self._centroids.items():
            sim = _cosine_sim(emb, centroid)
            if sim > best_sim:
                best_sim = sim
                best_id = fid
        if best_sim >= threshold and best_id >= 0:
            face = self.get_face(best_id)
            best_name = face["name"] if face else ""
            return (best_id, best_name, best_sim)
        return None

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------
    @property
    def num_faces(self) -> int:
        return self._conn.execute("SELECT COUNT(*) FROM faces").fetchone()[0]

    @property
    def db_path(self) -> str:
        return self._db_path

    def close(self) -> None:
        self._conn.close()
