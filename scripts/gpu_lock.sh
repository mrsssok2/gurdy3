#!/usr/bin/env bash
# Shared GPU / Gazebo / heavy-Docker lock for ALL projects on this machine (gurdy3, gurdy4, humanoid, ...): one lock file,
# ~/projects/.gpu_lock, taken ONCE for a whole batch (an overnight run, a series of tries), never per try.
# Identical copies live in each project (scripts/gpu_lock.sh); keep them identical (GPU_LOCK_VERSION).
#
#   source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:my_batch" || exit 1     # in a script: held until the script exits
#   scripts/gpu_lock.sh run OWNER -- COMMAND ARGS...                                 # wrap any command (exec: same pid)
#   scripts/gpu_lock.sh status                                                       # who holds it (owner, pid, since)
#
# How: a holder process `flock -o LOCK tail --pid=OWNER_PID` keeps the flock exactly as long as the owning script lives
# (also after kill -9: tail notices within ~1 s), so a dead owner never leaves a stale lock, and children (docker, gazebo)
# never inherit the lock fd. The holder writes LOCK.info (owner, pid, start) only after it got the lock and removes it on release. Waiters block (print
# the holder every minute); GPU_LOCK_TIMEOUT=S gives up after S seconds (default 0 = wait forever). Nested scripts of the same
# batch see GPU_LOCK_OWNER_PID (exported) alive and do not wait. DRY_RUN=1 (CPU control-flow tests) never takes the lock.
GPU_LOCK_VERSION=1
GPU_LOCK=${GPU_LOCK:-$HOME/projects/.gpu_lock}

gpu_lock_status() {     # -> prints "free" / "held by OWNER (pid P, since T)"; clears info left by a dead pid
  local info pid
  info=$(cat "$GPU_LOCK.info" 2>/dev/null)
  pid=$(sed -n 's/^pid=//p' <<< "$info")
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && ! flock -n "$GPU_LOCK" true 2>/dev/null; then
    echo "held by $(sed -n 's/^owner=//p' <<< "$info") (pid $pid, since $(sed -n 's/^start=//p' <<< "$info"))"
  elif ! flock -n "$GPU_LOCK" true 2>/dev/null; then
    echo "held (holder starting up)"
  else
    [ -n "$info" ] && { rm -f "$GPU_LOCK.info"; echo "free (cleared stale info of dead pid ${pid:-?})"; return; }
    echo "free"
  fi; }

gpu_lock_acquire() {    # gpu_lock_acquire OWNER [TIMEOUT_S] -> 0 when this process (or an ancestor of it) holds the lock
  local owner=${1:-unknown} timeout=${2:-${GPU_LOCK_TIMEOUT:-0}} me=$$ t0 holder last=0
  [ "${DRY_RUN:-0}" = 1 ] && return 0
  if [ -n "${GPU_LOCK_OWNER_PID:-}" ] && kill -0 "$GPU_LOCK_OWNER_PID" 2>/dev/null && \
     grep -qx "pid=$GPU_LOCK_OWNER_PID" "$GPU_LOCK.info" 2>/dev/null; then
    return 0                                                     # held by the batch that started us
  fi
  mkdir -p "$(dirname "$GPU_LOCK")"; touch "$GPU_LOCK"
  flock -o "$GPU_LOCK" sh -c 'printf "owner=%s\npid=%s\nstart=%s\nhost=%s\n" "$1" "$2" "$(date -u +%FT%TZ)" "$(hostname)" > "$3.info.tmp" &&
      mv "$3.info.tmp" "$3.info"; tail --pid="$2" -f /dev/null; rm -f "$3.info"' _ "$owner" "$me" "$GPU_LOCK" </dev/null >/dev/null 2>&1 &
  holder=$!
  t0=$(date +%s); last=$t0
  sleep 0.1
  grep -qx "pid=$me" "$GPU_LOCK.info" 2>/dev/null || echo "gpu_lock: $owner waiting, $(gpu_lock_status)" >&2
  until grep -qx "pid=$me" "$GPU_LOCK.info" 2>/dev/null; do
    kill -0 "$holder" 2>/dev/null || { echo "gpu_lock: holder process died" >&2; return 1; }
    local now; now=$(date +%s)
    if [ "$timeout" != 0 ] && [ $(( now - t0 )) -ge "$timeout" ]; then
      kill "$holder" 2>/dev/null; echo "gpu_lock: still $(gpu_lock_status) after ${timeout}s: giving up" >&2; return 1
    fi
    if [ $(( now - last )) -ge 60 ]; then echo "gpu_lock: $owner still waiting, $(gpu_lock_status)" >&2; last=$now; fi
    sleep 0.2
  done
  export GPU_LOCK_OWNER_PID=$me GPU_LOCK_HOLDER_PID=$holder
  echo "gpu_lock: acquired by $owner (pid $me) after $(( $(date +%s) - t0 )) s" >&2
  return 0; }

gpu_lock_release() {    # optional: release before the script ends (otherwise automatic at exit)
  [ -n "${GPU_LOCK_HOLDER_PID:-}" ] && [ "${GPU_LOCK_OWNER_PID:-}" = "$$" ] || return 0
  rm -f "$GPU_LOCK.info"; kill "$GPU_LOCK_HOLDER_PID" 2>/dev/null; wait "$GPU_LOCK_HOLDER_PID" 2>/dev/null
  unset GPU_LOCK_OWNER_PID GPU_LOCK_HOLDER_PID; }

if [ "${BASH_SOURCE[0]}" = "$0" ]; then          # command-line use
  case "${1:-}" in
    status) gpu_lock_status ;;
    run) shift; owner=${1:?usage: gpu_lock.sh run OWNER -- COMMAND...}; shift; [ "${1:-}" = "--" ] && shift
         gpu_lock_acquire "$owner" || exit 1; exec "$@" ;;
    *) echo "usage: $0 status | run OWNER -- COMMAND ARGS..."; exit 2 ;;
  esac
fi
