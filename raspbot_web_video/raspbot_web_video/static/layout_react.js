(function () {
  const STORAGE_KEY = 'raspbot.dashboard.layout.v1';

  function slugify(text) {
    return String(text || '')
      .toLowerCase()
      .replace(/[^a-z0-9]+/g, '-')
      .replace(/(^-|-$)/g, '') || 'panel';
  }

  function ensurePanelIds(cards) {
    const used = new Set();
    cards.forEach((card, idx) => {
      const titleEl = card.querySelector('h2');
      const title = titleEl ? titleEl.textContent.trim() : `Panel ${idx + 1}`;
      let baseId = card.id || `panel-${slugify(title)}`;
      let panelId = baseId;
      let i = 2;
      while (used.has(panelId)) {
        panelId = `${baseId}-${i++}`;
      }
      used.add(panelId);
      card.dataset.panelId = panelId;
      card.dataset.panelTitle = title;
      if (!card.id) {
        card.id = panelId;
      }
    });
  }

  function readStoredLayout() {
    try {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (!raw) return null;
      return JSON.parse(raw);
    } catch (e) {
      return null;
    }
  }

  function saveLayout(layout) {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(layout));
    } catch (e) {
      // ignore quota/storage errors
    }
  }

  function buildDefaults(cards) {
    return cards.map((card) => {
      const computed = window.getComputedStyle(card);
      const visible = computed.display !== 'none';
      const size = card.classList.contains('card-stream') ? 'wide' : 'normal';
      return {
        id: card.dataset.panelId,
        title: card.dataset.panelTitle || card.dataset.panelId,
        visible,
        size,
      };
    });
  }

  function normalizeLayout(layout, defaults) {
    const byId = new Map(defaults.map((d) => [d.id, d]));
    const out = [];

    if (Array.isArray(layout)) {
      layout.forEach((item) => {
        if (!item || typeof item.id !== 'string') return;
        const def = byId.get(item.id);
        if (!def) return;
        out.push({
          id: item.id,
          title: def.title,
          visible: typeof item.visible === 'boolean' ? item.visible : def.visible,
          size: (item.size === 'normal' || item.size === 'wide' || item.size === 'full') ? item.size : def.size,
        });
        byId.delete(item.id);
      });
    }

    for (const def of byId.values()) out.push(def);
    return out;
  }

  function applyLayoutState(layout, dashboard) {
    const byId = new Map();
    dashboard.querySelectorAll('.card').forEach((card) => {
      byId.set(card.dataset.panelId, card);
    });

    layout.forEach((item) => {
      const card = byId.get(item.id);
      if (!card) return;

      card.classList.remove('panel-span-1', 'panel-span-2', 'panel-span-3');
      if (item.size === 'normal') card.classList.add('panel-span-1');
      else if (item.size === 'wide') card.classList.add('panel-span-2');
      else card.classList.add('panel-span-3');

      card.style.display = item.visible ? '' : 'none';
      dashboard.appendChild(card);
    });
  }

  function bootstrapReactLayout() {
    const dashboard = document.querySelector('.dashboard');
    const header = document.querySelector('.header');
    if (!dashboard || !header) return;

    const cards = Array.from(dashboard.querySelectorAll('.card'));
    if (!cards.length) return;

    ensurePanelIds(cards);
    const defaults = buildDefaults(cards);

    const reactRootHost = document.createElement('div');
    reactRootHost.id = 'layoutRoot';
    header.insertAdjacentElement('afterend', reactRootHost);

    const e = window.React.createElement;
    const { useMemo, useState, useEffect } = window.React;

    function App() {
      const initial = useMemo(() => {
        const stored = readStoredLayout();
        return normalizeLayout(stored, defaults);
      }, []);

      const [open, setOpen] = useState(false);
      const [layout, setLayout] = useState(initial);

      useEffect(() => {
        applyLayoutState(layout, dashboard);
        saveLayout(layout);
      }, [layout]);

      function move(idx, dir) {
        const target = idx + dir;
        if (target < 0 || target >= layout.length) return;
        const next = layout.slice();
        const tmp = next[idx];
        next[idx] = next[target];
        next[target] = tmp;
        setLayout(next);
      }

      function setVisible(idx, visible) {
        const next = layout.slice();
        next[idx] = { ...next[idx], visible };
        setLayout(next);
      }

      function setSize(idx, size) {
        const next = layout.slice();
        next[idx] = { ...next[idx], size };
        setLayout(next);
      }

      function resetAll() {
        localStorage.removeItem(STORAGE_KEY);
        setLayout(defaults.map((d) => ({ ...d })));
      }

      return e('div', { className: 'layout-toolbar' },
        e('button', {
          className: 'primary',
          type: 'button',
          onClick: () => setOpen(!open),
          title: 'Open panel layout settings',
        }, open ? 'Hide Layout Settings' : 'Layout Settings'),
        e('button', {
          type: 'button',
          onClick: resetAll,
          title: 'Reset panel order, size, and visibility',
        }, 'Reset Layout'),
        e('div', { className: 'kv' }, 'Persistent per browser'),
        open ? e('div', { className: 'layout-list' },
          layout.map((item, idx) => e('div', { key: item.id, className: 'layout-item' },
            e('input', {
              type: 'checkbox',
              checked: item.visible,
              onChange: (ev) => setVisible(idx, ev.target.checked),
              title: 'Show / hide panel',
            }),
            e('span', { className: 'name', title: item.title }, item.title),
            e('select', {
              value: item.size,
              onChange: (ev) => setSize(idx, ev.target.value),
              title: 'Panel size',
            },
              e('option', { value: 'normal' }, 'Normal'),
              e('option', { value: 'wide' }, 'Wide'),
              e('option', { value: 'full' }, 'Full')
            ),
            e('button', {
              type: 'button',
              onClick: () => move(idx, -1),
              title: 'Move panel up',
            }, '↑'),
            e('button', {
              type: 'button',
              onClick: () => move(idx, 1),
              title: 'Move panel down',
            }, '↓')
          ))
        ) : null
      );
    }

    window.ReactDOM.createRoot(reactRootHost).render(e(App));
  }

  function init() {
    if (!window.React || !window.ReactDOM) {
      console.warn('[layout] React not available; layout manager disabled.');
      return;
    }
    bootstrapReactLayout();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
