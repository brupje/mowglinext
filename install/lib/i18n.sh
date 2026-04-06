#!/usr/bin/env bash

# ── Internationalisation ────────────────────────────────────────────────────
# Detects language from LANG env var or lets the user choose.
# Sources the matching locale file. Default: English.

LOCALE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../locale" && pwd)"

load_locale() {
  local lang="${MOWGLI_LANG:-}"

  # Auto-detect from system locale if not explicitly set
  if [[ -z "$lang" ]]; then
    case "${LANG:-en}" in
      fr*) lang="fr" ;;
      *)   lang="en" ;;
    esac
  fi

  local locale_file="${LOCALE_DIR}/${lang}.sh"
  if [[ ! -f "$locale_file" ]]; then
    locale_file="${LOCALE_DIR}/en.sh"
    lang="en"
  fi

  # shellcheck disable=SC1090
  source "$locale_file"
  MOWGLI_LANG="$lang"
}

select_language() {
  echo ""
  echo -e "${BOLD}Language / Langue:${NC}"
  echo "  1) English"
  echo "  2) Francais"
  prompt "Choice / Choix" "1"
  case "$REPLY" in
    2) MOWGLI_LANG="fr" ;;
    *) MOWGLI_LANG="en" ;;
  esac
  load_locale
}
