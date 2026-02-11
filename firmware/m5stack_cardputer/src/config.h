/*
 * Raspbot V2 — M5Stack Cardputer configuration
 *
 * These are compile-time FALLBACK defaults.  On first boot the Cardputer
 * runs a WiFi setup wizard (scan → select → type password → enter Raspbot
 * IP).  The chosen values are persisted in NVS flash and used on subsequent
 * boots.  To re-run the wizard, hold the G0 button while powering on.
 *
 * You can still hard-code values here; they will be used if NVS is empty.
 */

#pragma once

// ── WiFi credentials (fallback defaults) ────────────────────────────
static const char* DEFAULT_WIFI_SSID = "";
static const char* DEFAULT_WIFI_PASS = "";

// ── Raspbot web server (fallback defaults) ──────────────────────────
static const char* DEFAULT_RASPBOT_HOST = "192.168.1.100";
static constexpr int DEFAULT_RASPBOT_PORT = 8080;
