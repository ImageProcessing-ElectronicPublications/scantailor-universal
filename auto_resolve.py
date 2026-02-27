"""
Auto-resolve git merge conflicts for ScanTailor Universal.
For each conflict block, keeps BOTH sides (ours then theirs).
Handles the 'shared method boundary' pattern where void/} are
shared between two method definitions.

Usage: python auto_resolve.py [file1] [file2] ...
       python auto_resolve.py --all   (resolves all conflicted files)
"""
import os, sys, subprocess

def get_conflicted_files():
    """Get list of files with merge conflicts."""
    result = subprocess.run(
        ['git', 'diff', '--name-only', '--diff-filter=U'],
        capture_output=True, text=True
    )
    return [f.strip() for f in result.stdout.strip().split('\n') if f.strip()]

def resolve_file(path):
    """Resolve all conflict markers in a file by keeping both sides."""
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        lines = f.readlines()

    out = []
    i = 0
    conflict_count = 0

    while i < len(lines):
        line = lines[i]

        if line.startswith('<<<<<<<'):
            conflict_count += 1

            # Collect ours block
            ours = []
            i += 1
            while i < len(lines) and not lines[i].startswith('======='):
                ours.append(lines[i])
                i += 1
            i += 1  # skip =======

            # Collect theirs block
            theirs = []
            while i < len(lines) and not lines[i].startswith('>>>>>>>'):
                theirs.append(lines[i])
                i += 1
            i += 1  # skip >>>>>>>

            # Detect 'shared method boundary' pattern:
            # The line BEFORE <<<<<<< is a return type (void, QImage, etc.)
            # The line AFTER >>>>>>> is a closing brace }
            # This means two separate methods were added at the same location.
            prev_line = out[-1].rstrip() if out else ''
            next_line = lines[i].rstrip() if i < len(lines) else ''

            method_types = {'void', 'QImage', 'GrayImage', 'BinaryImage',
                          'static void', 'static QImage', 'static GrayImage'}
            is_method_boundary = (
                prev_line in method_types and
                next_line == '}'
            )

            if is_method_boundary:
                # Two methods sharing a type prefix and closing brace.
                # ours = first method body (without })
                # theirs = second method body (without })
                # Emit: ours + } + blank + type + theirs (shared } closes theirs)
                out.extend(ours)
                out.append('}\n')
                out.append('\n')
                out.append(prev_line + '\n')  # repeat the return type
                out.extend(theirs)
            else:
                # Default: keep both blocks sequentially
                out.extend(ours)
                out.extend(theirs)
        else:
            out.append(line)
            i += 1

    # Verify no markers remain
    result_text = ''.join(out)
    remaining = result_text.count('<<<<<<<') + result_text.count('>>>>>>>')
    if remaining > 0:
        print(f"  WARNING: {remaining} conflict markers remain in {path}")

    with open(path, 'w', encoding='utf-8') as f:
        f.writelines(out)

    return conflict_count

def main():
    if '--all' in sys.argv:
        files = get_conflicted_files()
    else:
        files = sys.argv[1:]

    if not files:
        print("No conflicted files found.")
        return

    total = 0
    for f in files:
        if os.path.exists(f):
            n = resolve_file(f)
            total += n
            print(f"  Resolved {n} conflict(s) in {f}")
        else:
            print(f"  NOT FOUND: {f}")

    print(f"Total: {total} conflicts resolved in {len(files)} file(s)")

if __name__ == '__main__':
    main()
