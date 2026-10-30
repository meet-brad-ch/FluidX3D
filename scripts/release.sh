#!/usr/bin/env bash
# Prepares a FluidX3D-API release: checks the version against the upstream base and the changelog, then makes the
# annotated tag v<version>. It pushes nothing: push the tag when the release is approved, and the workflow
# .github/workflows/release-api.yml makes the GitHub release from the changelog section.
# Usage: scripts/release.sh <upstream version>-api.<n>, for example scripts/release.sh 3.8-api.1
# Needs the remote "upstream" (ProjectPhysX/FluidX3D) with its tags.
set -euo pipefail
version="${1:?usage: release.sh <upstream version>-api.<n>, for example 3.8-api.1}"
if ! [[ "$version" =~ ^([0-9]+\.[0-9]+)-api\.([1-9][0-9]*)$ ]]; then
  echo "release: '$version' is not <upstream version>-api.<n>, for example 3.8-api.1" >&2
  exit 1
fi
base_version="${BASH_REMATCH[1]}"
repo="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo"
[ -z "$(git status --porcelain)" ] || { echo "release: working tree is not clean" >&2; exit 1; }
git fetch -q upstream --tags
fork_point="$(git merge-base HEAD upstream/master)"
base_tag="$(git describe --tags --abbrev=0 "$fork_point")"
if [ "$base_tag" != "v$base_version" ]; then
  echo "release: the fork is based on upstream $base_tag, not v$base_version" >&2
  exit 1
fi
if ! grep -q "^## \[$version\]" CHANGELOG.md; then
  echo "release: CHANGELOG.md has no section [$version]" >&2
  exit 1
fi
if git rev-parse -q --verify "refs/tags/v$version" >/dev/null; then
  echo "release: the tag v$version exists already" >&2
  exit 1
fi
git tag -a "v$version" -m "FluidX3D-API $version (on FluidX3D $base_tag, upstream $(git rev-parse --short "$fork_point"))"
echo "tagged v$version; push with: git push origin v$version"
