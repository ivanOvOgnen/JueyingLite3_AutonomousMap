#!/usr/bin/env bash
# Install a per-user cron entry to start the Lite3 web API after reboot.
# Does not require root and does not alter vendor service units.
set -eo pipefail

workspace="$HOME/lite3_autonomy_ws"
launcher="$workspace/scripts/start_robot_api.sh"
log_file="$workspace/logs/robot_api_autostart.log"
if [[ ! -r "$launcher" || ! -r "$workspace/install/setup.bash" ]]; then
  echo "Project launcher/build is missing: $launcher" >&2
  exit 1
fi

mkdir -p "$workspace/logs"
cron_line="@reboot sleep 45; /bin/bash $launcher >> $log_file 2>&1"
current="$(crontab -l 2>/dev/null || true)"
if grep -Fqx "$cron_line" <<<"$current"; then
  echo "Lite3 API boot start is already installed for user $USER."
  exit 0
fi
{
  if [[ -n "$current" ]]; then
    printf '%s\n' "$current"
  fi
  printf '%s\n' "$cron_line"
} | crontab -
echo "Installed user-level @reboot start for the Lite3 web API."
echo "It starts after the next reboot; it does not start the API now."
