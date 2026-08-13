#!/usr/bin/env bash

set -u -o pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
script_path="$repo_root/$(basename "${BASH_SOURCE[0]}")"
log_dir="${LOG_DIR:-$repo_root/_build/example-build-logs}"
build_cmd="${BUILD_CMD:-idf.py build}"
jobs="${JOBS:-1}"

shopt -s nullglob

usage() {
  cat <<'EOF'
Usage: ./build_examples.sh [-j JOBS] [component ...]

Build all component examples, or only the named components.

Options:
  -j, --jobs JOBS   Number of examples to build in parallel (default: 1)
  -h, --help        Show this help text

Environment:
  BUILD_CMD         Build command to run in each example directory
  JOBS              Default parallel job count
  LOG_DIR           Directory for per-component build logs
EOF
}

run_one() {
  local component="$1"
  local status_file="$2"
  local example_dir="$repo_root/components/$component/example"
  local log_file="$log_dir/$component.log"

  rm -rf "$example_dir/build"

  if (
    cd "$example_dir" &&
    bash -lc "$build_cmd"
  ) >"$log_file" 2>&1; then
    printf 'PASS\t%s\n' "$component" >>"$status_file"
    echo "PASS: $component"
    return 0
  fi

  printf 'FAIL\t%s\n' "$component" >>"$status_file"
  echo "FAIL: $component"
  return 1
}

if [[ "${1:-}" == "--run-one" ]]; then
  if [[ $# -ne 3 ]]; then
    echo "Worker mode expects: --run-one STATUS_FILE COMPONENT" >&2
    exit 2
  fi
  run_one "$3" "$2"
  exit $?
fi

declare -a requested_components=()
declare -a selected_components=()
declare -a failed_components=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -j|--jobs)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for $1" >&2
        exit 2
      fi
      jobs="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      while [[ $# -gt 0 ]]; do
        requested_components+=("$1")
        shift
      done
      ;;
    *)
      requested_components+=("$1")
      shift
      ;;
  esac
done

if ! [[ "$jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "JOBS must be a positive integer." >&2
  exit 2
fi

if [[ ${#requested_components[@]} -gt 0 ]]; then
  for component in "${requested_components[@]}"; do
    if [[ -d "$repo_root/components/$component/example" ]]; then
      selected_components+=("$component")
    else
      echo "Skipping unknown component: $component" >&2
    fi
  done
else
  for example_dir in "$repo_root"/components/*/example; do
    selected_components+=("$(basename "$(dirname "$example_dir")")")
  done
fi

if [[ ${#selected_components[@]} -eq 0 ]]; then
  echo "No component examples selected." >&2
  exit 2
fi

mkdir -p "$log_dir"
status_file="$(mktemp "${TMPDIR:-/tmp}/build_examples_status.XXXXXX")"
trap 'rm -f "$status_file"' EXIT

echo "Building ${#selected_components[@]} component example(s)"
echo "Build command: $build_cmd"
echo "Parallel jobs: $jobs"
echo "Logs: $log_dir"
echo

if [[ "$jobs" -eq 1 ]]; then
  total_count=${#selected_components[@]}
  index=0
  for component in "${selected_components[@]}"; do
    index=$((index + 1))
    echo "==> [$index/$total_count] $component"
    run_one "$component" "$status_file"
    echo
  done
else
  printf '%s\0' "${selected_components[@]}" | xargs -0 -n 1 -P "$jobs" "$script_path" --run-one "$status_file"
fi

pass_count=0
fail_count=0

while IFS=$'\t' read -r status component; do
  [[ -n "${status:-}" ]] || continue
  if [[ "$status" == "PASS" ]]; then
    pass_count=$((pass_count + 1))
  else
    fail_count=$((fail_count + 1))
    failed_components+=("$component")
  fi
done <"$status_file"

echo "=== SUMMARY ==="
echo "PASSED: $pass_count"
echo "FAILED: $fail_count"

if [[ $fail_count -gt 0 ]]; then
  echo
  echo "Failed components:"
  for component in "${failed_components[@]}"; do
    echo "- $component (log: $log_dir/$component.log)"
  done
  exit 1
fi

echo
echo "All component examples built successfully."
