#!/usr/bin/env python3
import urllib.request
import time
import json
import sys
import argparse

def test_endpoint(url, iterations=10):
    times = []
    success_count = 0
    failures = 0
    
    print(f"Testing {url} ({iterations} iterations)...")
    
    for i in range(iterations):
        start = time.monotonic()
        try:
            with urllib.request.urlopen(url, timeout=2.0) as response:
                if response.status == 200:
                    data = response.read()
                    success_count += 1
                    # Parse JSON to verify validity
                    json.loads(data)
                else:
                    failures += 1
        except Exception as e:
            failures += 1
            print(f"  Fail: {e}")
        
        duration = (time.monotonic() - start) * 1000.0
        times.append(duration)
        time.sleep(0.05) # slight throttle
        
    avg_time = sum(times) / len(times) if times else 0
    print(f"  Avg Latency: {avg_time:.2f} ms")
    print(f"  Success: {success_count}/{iterations}")
    return avg_time

def monitor_status(url, duration_sec=5):
    print(f"Monitoring status FPS for {duration_sec}s...")
    start_time = time.monotonic()
    last_frame_count = -1
    frames_seen = 0
    
    while (time.monotonic() - start_time) < duration_sec:
        try:
            with urllib.request.urlopen(url, timeout=1.0) as response:
                data = json.loads(response.read())
                fc = data.get('frame_count', 0)
                if last_frame_count >= 0:
                    diff = fc - last_frame_count
                    if diff > 0:
                        frames_seen += diff
                last_frame_count = fc
        except Exception:
            pass
        time.sleep(0.5)

    fps = frames_seen / duration_sec
    print(f"  Estimated MJPEG Stream FPS: {fps:.2f}")

def main():
    parser = argparse.ArgumentParser(description="Verify Raspbot Web Video Performance")
    parser.add_argument("--host", default="localhost", help="Hostname of the robot")
    parser.add_argument("--port", default=8080, type=int, help="Port")
    args = parser.parse_args()
    
    base_url = f"http://{args.host}:{args.port}"
    
    print(f"Connecting to {base_url}...")
    
    # 1. Test /api/fast (Optimized Detection+IMU)
    # This endpoint used to do heavy serialization. Now should be fast.
    test_endpoint(f"{base_url}/api/fast", iterations=20)
    
    # 2. Test /status (General health)
    test_endpoint(f"{base_url}/status", iterations=5)
    
    # 3. Monitor FPS via status
    monitor_status(f"{base_url}/status", duration_sec=5)
    
    # 4. Test /detections (Optimized raw JSON)
    test_endpoint(f"{base_url}/detections", iterations=10)

if __name__ == "__main__":
    main()
