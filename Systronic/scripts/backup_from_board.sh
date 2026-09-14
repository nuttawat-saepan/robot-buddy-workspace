#!/bin/bash
# Pull everything off the board that cannot be made again.
#
# Read-only on the board: it copies and deletes nothing. Run it after every
# map, after every block of test cases, and once more before the board is
# wiped - not only at the end of the day. On 2026-09-07 the board was cleared
# with onsite.env and the photographs still on it, and on 2026-09-14 a map was
# lost to a deploy an hour after it was made.
#
# Bags are the priority. A map can be rebuilt from a bag; a bag cannot be
# rebuilt without walking the room again.
#
#     ./scripts/backup_from_board.sh unitree@192.168.80.111
#     ./scripts/backup_from_board.sh unitree@192.168.80.111 ~/somewhere_else
set +u

TARGET="${1:-}"
if [ -z "$TARGET" ]; then
    echo "usage: backup_from_board.sh <user@host> [dest]"
    exit 2
fi
DEST="${2:-$HOME/site_backup_$(date +%Y%m%d)}"
mkdir -p "$DEST"/{maps,bags,captures,config,testruns}

echo "== backing up $TARGET into $DEST"

# onsite.env first: it is the smallest, the easiest to lose, and the one that
# takes longest to reconstruct from memory at the next site.
echo "-- onsite.env"
scp -q "$TARGET:~/go2_ws/scripts/onsite.env" "$DEST/config/onsite.env.board" \
    2>/dev/null && echo "   ok" || echo "   none found"

echo "-- maps"
scp -q "$TARGET:~/maps/*" "$DEST/maps/" 2>/dev/null
scp -q "$TARGET:~/go2_ws/src/go2_control/map/site_*" "$DEST/maps/" 2>/dev/null
ls "$DEST/maps/" 2>/dev/null | sed 's/^/   /' || echo "   none found"

echo "-- photographs"
scp -qr "$TARGET:~/go2_captures/*" "$DEST/captures/" 2>/dev/null
echo "   $(ls "$DEST/captures" 2>/dev/null | wc -l) file(s)"

echo "-- test runs"
scp -qr "$TARGET:~/go2_testruns/*" "$DEST/testruns/" 2>/dev/null
echo "   $(ls "$DEST/testruns" 2>/dev/null | wc -l) run(s)"

# Last and slowest, with --progress, because this is the one worth watching
# and the one to leave running while other work continues. rsync rather than
# scp so an interrupted transfer resumes instead of starting over.
echo "-- bags (slow, resumable)"
rsync -az --partial --progress "$TARGET:~/bags/" "$DEST/bags/" 2>/dev/null \
    || echo "   none found"

echo
echo "== $DEST"
du -sh "$DEST"/* 2>/dev/null
