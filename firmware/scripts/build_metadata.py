from __future__ import annotations

from datetime import datetime, timezone
import subprocess

Import("env")


def git_output(*args: str) -> str:
    try:
        return subprocess.check_output(
            ["git", *args],
            cwd=env.subst("$PROJECT_DIR"),
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return "unknown"


sha = git_output("rev-parse", "--short=12", "HEAD")
status = git_output("status", "--porcelain")
dirty = "1" if status not in ("", "unknown") else "0"
build_date = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")

env.Append(
    CPPDEFINES=[
        ("DAFT_GIT_SHA", f'\\"{sha}\\"'),
        ("DAFT_BUILD_DATE", f'\\"{build_date}\\"'),
        ("DAFT_GIT_DIRTY", dirty),
    ]
)
