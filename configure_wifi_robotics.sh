#!/usr/bin/env bash
# =============================================================================
# configure_wifi_robotics.sh
#
# Adds the Robotics WiFi hotspot to the Raspberry Pi's known networks.
# Uses wpa_supplicant for headless/CLI configuration.
#
# Usage:
#   sudo ./configure_wifi_robotics.sh
#
# =============================================================================

set -euo pipefail

# Configuration
SSID="Robotics"
PSK="robotics@100"
WPA_CONF="/etc/wpa_supplicant/wpa_supplicant.conf"

# Colors
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
NC='\033[0m'

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo -e "${RED}Error: This script must be run as root (use sudo)${NC}"
   exit 1
fi

# Check if wpa_supplicant config exists
if [[ ! -f "$WPA_CONF" ]]; then
   echo -e "${RED}Error: $WPA_CONF not found${NC}"
   exit 1
fi

# Backup original config
BACKUP_FILE="${WPA_CONF}.backup.$(date +%s)"
cp "$WPA_CONF" "$BACKUP_FILE"
echo -e "${YLW}Backed up original config to: $BACKUP_FILE${NC}"

# Check if network already exists
if grep -q "ssid=\"$SSID\"" "$WPA_CONF"; then
   echo -e "${YLW}Network '$SSID' already exists in configuration${NC}"
   exit 0
fi

# Add the network using wpa_cli
echo -e "${GRN}Adding WiFi network: $SSID${NC}"

# Use wpa_cli to add the network (safer than manual editing)
NETWORK_ID=$(wpa_cli add_network | tail -1)
wpa_cli set_network "$NETWORK_ID" ssid "\"$SSID\""
wpa_cli set_network "$NETWORK_ID" psk "\"$PSK\""
wpa_cli set_network "$NETWORK_ID" key_mgmt WPA-PSK
wpa_cli enable_network "$NETWORK_ID"
wpa_cli save_config

echo -e "${GRN}✓ Successfully added WiFi network!${NC}"
echo -e "${GRN}✓ Network ID: $NETWORK_ID${NC}"
echo -e ""
echo "Next steps:"
echo "  1. Reconnect to WiFi using your system settings"
echo "  2. Or manually connect with: sudo wpa_cli select_network $NETWORK_ID"
echo "  3. Verify connection: iwconfig or nmcli device wifi list"
echo ""
echo -e "${YLW}Backup created at: $BACKUP_FILE${NC}"
