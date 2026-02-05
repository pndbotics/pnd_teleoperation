#!/usr/bin/env bash
# PND Retarget CLI setup script.
# Usage: source setup_cli.bash
# After sourcing: pteleop (and spteleop for root) with Typer tab completion.
# Prerequisites: uv installed, and uv sync run in project root.
#

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "[ERROR] Source this script instead of executing: source setup_cli.bash" >&2
  exit 1
fi

_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export PND_RETARGET_ROOT="${_PROJECT_ROOT}"

if ! command -v uv &>/dev/null; then
  echo "[ERROR] uv not found. Install it first, e.g.:" >&2
  echo "  pip install uv --user" >&2
  return 1 2>/dev/null || exit 1
fi

if [[ ! -d "${_PROJECT_ROOT}/.venv/bin" ]]; then
  echo "[ERROR] Virtual env not found: ${_PROJECT_ROOT}/.venv. Run in project root:" >&2
  echo "  uv sync" >&2
  return 1 2>/dev/null || exit 1
fi

#######################################
# Activate virtual environment
#######################################
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  deactivate 2>/dev/null || true
fi
# shellcheck source=/dev/null
if ! source "${_PROJECT_ROOT}/.venv/bin/activate" 2>/dev/null; then
  export PATH="${_PROJECT_ROOT}/.venv/bin:${PATH}"
fi
export PATH="${_PROJECT_ROOT}/.venv/bin:${PATH}"

if ! command -v pteleop &>/dev/null; then
  echo "[ERROR] pteleop not found. Run in project root: uv sync" >&2
  return 1 2>/dev/null || exit 1
fi

#######################################
# pteleop Typer completion
#######################################
_COMPLETION_FILE="${_PROJECT_ROOT}/.pteleop-completion.bash"
_NEED_REGEN=false
if [[ ! -s "${_COMPLETION_FILE}" ]]; then
  _NEED_REGEN=true
elif [[ -f "${_PROJECT_ROOT}/scripts/cli/pteleop.py" ]] && [[ "${_PROJECT_ROOT}/scripts/cli/pteleop.py" -nt "${_COMPLETION_FILE}" ]]; then
  _NEED_REGEN=true
fi
if [[ "${_NEED_REGEN}" == "true" ]]; then
  echo "[INFO] Generating spteleop completion script..." >&2
  if pteleop --show-completion bash > "${_COMPLETION_FILE}" 2>/dev/null; then
    chmod +x "${_COMPLETION_FILE}" 2>/dev/null || true
  fi
fi
if [[ -s "${_COMPLETION_FILE}" ]]; then
  # shellcheck source=/dev/null
  if source "${_COMPLETION_FILE}" 2>/dev/null; then
    if [[ -z "${PND_RETARGET_SETUP_LOADED:-}" ]]; then
      echo "[SUCCESS] spteleop completion active; press Tab after 'spteleop '" >&2
      export PND_RETARGET_SETUP_LOADED=1
    fi
  fi
fi

#######################################
# spteleop wrapper (sudo pteleop)
#######################################
_SPTELEOP_PATH="${_PROJECT_ROOT}/.venv/bin/spteleop"
if [[ ! -f "${_SPTELEOP_PATH}" ]]; then
  echo "[INFO] Creating spteleop wrapper..." >&2
  cat <<'EOF' > "${_SPTELEOP_PATH}"
#!/usr/bin/env bash
_REAL_CMD="$(dirname "$0")/pteleop"
sudo -E "${_REAL_CMD}" "$@"
EOF
  chmod +x "${_SPTELEOP_PATH}"
fi
if declare -F _pteleop_completion &>/dev/null; then
  complete -o default -F _pteleop_completion spteleop
fi

unset _PROJECT_ROOT _COMPLETION_FILE _NEED_REGEN _SPTELEOP_PATH
