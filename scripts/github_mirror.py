"""Copy the approved GitLab branches to GitHub without rewriting either history."""

import base64
import os
import subprocess
import sys
from pathlib import Path

BRANCHES = ("develop", "master")
DESTINATION = "https://github.com/yoon1967/Chill-Drone.git"
SOURCE_NAMESPACE = "refs/github-mirror-source"


def git(repo, *args, env=None):
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
        timeout=180,
    )
    if result.returncode:
        # Git/HTTP diagnostics can include credentials or source repository URLs.
        raise RuntimeError(
            f"git {args[0]} failed (exit {result.returncode}). "
            "Check authentication, repository rules, and divergent history."
        )
    return result.stdout.strip()


def github_environment(token):
    env = os.environ.copy()
    for key in tuple(env):
        if key.startswith(("GIT_TRACE", "GIT_CONFIG_KEY_", "GIT_CONFIG_VALUE_")):
            env.pop(key)
    for key in ("GIT_CURL_VERBOSE", "GIT_ASKPASS", "SSH_ASKPASS", "GIT_CONFIG_PARAMETERS"):
        env.pop(key, None)
    header = base64.b64encode(f"x-access-token:{token}".encode()).decode()
    env.update(
        GIT_TERMINAL_PROMPT="0",
        GCM_INTERACTIVE="Never",
        GIT_CONFIG_COUNT="3",
        GIT_CONFIG_KEY_0="credential.helper",
        GIT_CONFIG_VALUE_0="",
        GIT_CONFIG_KEY_1="http.extraHeader",
        GIT_CONFIG_VALUE_1="",
        GIT_CONFIG_KEY_2="http.https://github.com/.extraHeader",
        GIT_CONFIG_VALUE_2=f"Authorization: Basic {header}",
    )
    return env


def mirror(repo, destination=DESTINATION, token=None):
    """Fetch current source tips, atomically fast-forward both branches, verify SHA."""
    git(
        repo,
        "fetch",
        "--no-tags",
        "--no-write-fetch-head",
        "origin",
        *(f"+refs/heads/{branch}:{SOURCE_NAMESPACE}/{branch}" for branch in BRANCHES),
    )
    expected = {
        branch: git(repo, "rev-parse", "--verify", f"{SOURCE_NAMESPACE}/{branch}^{{commit}}")
        for branch in BRANCHES
    }
    destination_env = github_environment(token) if token else None
    git(
        repo,
        "push",
        "--atomic",
        destination,
        *(f"{expected[branch]}:refs/heads/{branch}" for branch in BRANCHES),
        env=destination_env,
    )
    refs = git(repo, "ls-remote", "--heads", destination, *BRANCHES, env=destination_env)
    actual = {ref.removeprefix("refs/heads/"): sha for sha, ref in map(str.split, refs.splitlines())}
    if actual != expected:
        raise RuntimeError("GitHub branch SHA verification failed; retry after checking concurrent updates.")
    return expected


def main():
    if os.environ.get("CI_DEBUG_TRACE", "").lower() == "true":
        raise RuntimeError("Disable CI_DEBUG_TRACE before running the authenticated mirror.")
    token = os.environ.get("GITHUB_MIRROR_TOKEN")
    if not token:
        raise RuntimeError("Set the protected, hidden GITHUB_MIRROR_TOKEN variable for github-mirror.")
    for branch, sha in mirror(Path(__file__).resolve().parents[1], token=token).items():
        print(f"Verified GitHub {branch}: {sha}")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.TimeoutExpired) as error:
        message = "Git operation timed out." if isinstance(error, subprocess.TimeoutExpired) else str(error)
        print(message, file=sys.stderr)
        sys.exit(1)
