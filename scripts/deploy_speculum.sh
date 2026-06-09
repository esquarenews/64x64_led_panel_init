#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd -- "$SCRIPT_DIR/.." && pwd)}"
APP_DIR="${APP_DIR:-$REPO_DIR/speculum}"
SERVICE_NAME="${SERVICE_NAME:-speculum}"
ENV_DIR="${ENV_DIR:-/etc/speculum}"
ENV_FILE="${ENV_FILE:-$ENV_DIR/speculum.env}"
BRANCH="${BRANCH:-$(git -C "$REPO_DIR" branch --show-current 2>/dev/null || echo main)}"
RAILS_ENV="${RAILS_ENV:-production}"
BIND="${BIND:-0.0.0.0}"
PORT="${PORT:-5000}"
BUNDLE_BIN_DIR="${BUNDLE_BIN_DIR:-$(ruby -e 'puts Gem.user_dir' 2>/dev/null)/bin}"
SERVICE_PATH="${SERVICE_PATH:-$BUNDLE_BIN_DIR:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin}"
INSTALL_SERVICE="${INSTALL_SERVICE:-1}"
RUN_TESTS="${RUN_TESTS:-0}"
ALLOW_DIRTY="${ALLOW_DIRTY:-0}"
RESET_CREDENTIALS="${RESET_CREDENTIALS:-0}"
SPECULUM_USERNAME="${SPECULUM_USERNAME:-admin}"
SPECULUM_PASSWORD="${SPECULUM_PASSWORD:-}"

export PATH="$SERVICE_PATH:$PATH"

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

log() {
  printf '\n== %s ==\n' "$1"
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    printf 'Missing required command: %s\n' "$1" >&2
    exit 1
  fi
}

quote_for_env() {
  local value="$1"
  value="${value//\\/\\\\}"
  value="${value//\"/\\\"}"
  printf '"%s"' "$value"
}

ensure_env_file() {
  log "Preparing environment file"
  "${SUDO[@]}" mkdir -p "$ENV_DIR"

  if [[ ! -f "$ENV_FILE" ]]; then
    local secret password
    secret="$(cd "$APP_DIR" && SECRET_KEY_BASE_DUMMY=1 bundle exec rails secret)"
    password="${SPECULUM_PASSWORD:-$(openssl rand -base64 24 | tr -d '\n')}"

    local tmp
    tmp="$(mktemp)"
    cat >"$tmp" <<ENV
RAILS_ENV=$RAILS_ENV
RAILS_LOG_LEVEL=info
SECRET_KEY_BASE=$(quote_for_env "$secret")
SPECULUM_USERNAME=$(quote_for_env "$SPECULUM_USERNAME")
SPECULUM_PASSWORD=$(quote_for_env "$password")
SPECULUM_RUBY=$(quote_for_env "$(command -v ruby)")
BIND=$BIND
PORT=$PORT
RAILS_SERVE_STATIC_FILES=1
ENV
    "${SUDO[@]}" install -m 600 "$tmp" "$ENV_FILE"
    rm -f "$tmp"
    printf 'Created %s\n' "$ENV_FILE"
    printf 'Initial Speculum login: %s / %s\n' "$SPECULUM_USERNAME" "$password"
  else
    printf 'Using existing %s\n' "$ENV_FILE"
    update_runtime_env
    if [[ "$RESET_CREDENTIALS" == "1" ]]; then
      reset_credentials
    fi
  fi
}

update_runtime_env() {
  local tmp
  tmp="$(mktemp)"
  "${SUDO[@]}" grep -v -E '^(RAILS_ENV|BIND|PORT|RAILS_SERVE_STATIC_FILES|SPECULUM_RUBY)=' "$ENV_FILE" >"$tmp" || true
  cat >>"$tmp" <<ENV
RAILS_ENV=$RAILS_ENV
SPECULUM_RUBY=$(quote_for_env "$(command -v ruby)")
BIND=$BIND
PORT=$PORT
RAILS_SERVE_STATIC_FILES=1
ENV

  "${SUDO[@]}" install -m 600 "$tmp" "$ENV_FILE"
  rm -f "$tmp"
  printf 'Updated runtime settings in %s\n' "$ENV_FILE"
}

reset_credentials() {
  if [[ -z "$SPECULUM_PASSWORD" ]]; then
    printf 'RESET_CREDENTIALS=1 requires SPECULUM_PASSWORD to be set.\n' >&2
    exit 1
  fi

  local tmp
  tmp="$(mktemp)"
  "${SUDO[@]}" grep -v -E '^(SPECULUM_USERNAME|SPECULUM_PASSWORD)=' "$ENV_FILE" >"$tmp" || true
  printf 'SPECULUM_USERNAME=%s\n' "$(quote_for_env "$SPECULUM_USERNAME")" >>"$tmp"
  printf 'SPECULUM_PASSWORD=%s\n' "$(quote_for_env "$SPECULUM_PASSWORD")" >>"$tmp"

  "${SUDO[@]}" install -m 600 "$tmp" "$ENV_FILE"
  rm -f "$tmp"
  printf 'Updated Speculum login for %s in %s\n' "$SPECULUM_USERNAME" "$ENV_FILE"
}

install_systemd_service() {
  [[ "$INSTALL_SERVICE" == "1" ]] || return 0

  log "Installing systemd service"
  local unit_path="/etc/systemd/system/${SERVICE_NAME}.service"
  local tmp
  tmp="$(mktemp)"

  cat >"$tmp" <<UNIT
[Unit]
Description=Speculum LED panel controller
After=network.target

[Service]
Type=simple
WorkingDirectory=$APP_DIR
EnvironmentFile=$ENV_FILE
Environment=PATH=$SERVICE_PATH
ExecStart=/usr/bin/bash -lc 'exec bundle exec rails server -e "\${RAILS_ENV:-production}" -b "\${BIND:-0.0.0.0}" -p "\${PORT:-5000}"'
Restart=always
RestartSec=5
TimeoutStopSec=20
KillSignal=SIGTERM
User=$(id -un)
Group=$(id -gn)

[Install]
WantedBy=multi-user.target
UNIT

  "${SUDO[@]}" install -m 644 "$tmp" "$unit_path"
  rm -f "$tmp"
  "${SUDO[@]}" systemctl daemon-reload
  "${SUDO[@]}" systemctl enable "$SERVICE_NAME"
}

restart_service() {
  if [[ "$INSTALL_SERVICE" == "1" ]] && command -v systemctl >/dev/null 2>&1; then
    log "Restarting $SERVICE_NAME"
    "${SUDO[@]}" systemctl restart "$SERVICE_NAME"
    "${SUDO[@]}" systemctl --no-pager --lines=20 status "$SERVICE_NAME"
  else
    log "Starting app without systemd"
    cd "$APP_DIR"
    RAILS_ENV="$RAILS_ENV" BIND="$BIND" PORT="$PORT" bundle exec rails server -e "$RAILS_ENV" -b "$BIND" -p "$PORT"
  fi
}

require_command git
require_command ruby
require_command bundle
require_command openssl

log "Deploying Speculum from $REPO_DIR"
cd "$REPO_DIR"

if [[ "$ALLOW_DIRTY" != "1" ]] && [[ -n "$(git status --porcelain)" ]]; then
  git status --short
  printf '\nRefusing to deploy with local changes. Commit/stash them, or run ALLOW_DIRTY=1 %s.\n' "$0" >&2
  exit 1
fi

log "Updating git checkout"
git fetch origin "$BRANCH"
git pull --ff-only origin "$BRANCH"

log "Installing Ruby dependencies"
cd "$APP_DIR"
bundle config set path vendor/bundle
if [[ "$RUN_TESTS" == "1" ]]; then
  bundle config set without "development"
else
  bundle config set without "development test"
fi
bundle install

log "Preparing Rails runtime"
mkdir -p storage/runtime log tmp/pids
RAILS_ENV="$RAILS_ENV" SECRET_KEY_BASE_DUMMY=1 bundle exec rails assets:precompile
RAILS_ENV="$RAILS_ENV" SECRET_KEY_BASE_DUMMY=1 bundle exec rails speculum:thumbnails:warm

if [[ "$RUN_TESTS" == "1" ]]; then
  log "Running tests"
  RAILS_ENV=test bundle exec rails test
fi

ensure_env_file
install_systemd_service
restart_service

log "Deployment complete"
