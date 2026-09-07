# Codex runbook: synchronize the KiCad autorouter from Freerouting

This is the operating procedure for a Codex task that updates KiCad's native autorouter from
the upstream [Freerouting repository](https://github.com/freerouting/freerouting). It is written
as an execution contract, not as a general design description. A Codex agent must follow it
before changing `pcbnew/autorouter/`.

The goal is to preserve the Freerouting routing algorithms and their file/class structure where
practical while keeping KiCad's native board, geometry, DRC, transaction, and editor boundaries.
Only autorouter/engine behavior is in scope. Freerouting GUI, REST/API, management, Specctra I/O,
and other application-layer changes are not to be copied into KiCad.

## Non-negotiable rules

1. **GitHub is the upstream source of truth.** Read PR metadata, exact commit IDs, changed files,
   and patches from `https://github.com/freerouting/freerouting`. Do not use the local Freerouting
   checkout as the input to a synchronization run. A temporary clone fetched directly from GitHub
   is preferred. The local checkout may be inspected only as a read-only historical comparison
   when the user explicitly asks for that comparison.
2. **Never modify the reference checkout.** Do not check out branches, apply patches, build in
   place, generate files, or commit in `/Users/jmcasler/Documents/GitHub/mchamster/freerouting`.
3. **Pin every claim to immutable GitHub objects.** Record the upstream PR number and URL, the
   PR base SHA, head SHA, merge SHA when available, and the upstream default-branch tip used for
   the run. Never rely on a branch name, a mutable tag, or a PR title as an identity.
4. **Account for every PR in the selected range.** A PR may be ported, skipped with a reason,
   superseded, pending because it is still open, or blocked. It must never disappear because it
   looked unrelated at first glance.
5. **Port semantics, not Java or GUI plumbing.** Do not cherry-pick Java commits into KiCad and
   do not copy Freerouting GUI classes. Read the exact patch, map it through `UPSTREAM.md`, and
   make the smallest native C++ change that preserves the algorithmic behavior.
6. **Do not advance the synchronization baseline early.** Update the reference commit in
   `autorouter-docs/UPSTREAM.md` only after all PRs in the run have a ledger entry and the native
   implementation has passed the required validation.
7. **Do not stage, commit, push, or open a target PR unless the user explicitly requests that
   action.** When a PR is requested, include the upstream links and immutable SHAs described below.

## Repository roles

| Role | Location or URL | Rule |
|---|---|---|
| KiCad target | the current workspace, with native code under `pcbnew/autorouter/` | This is the only source tree Codex may edit. |
| Freerouting source | <https://github.com/freerouting/freerouting> | Fetch directly from GitHub for every sync. |
| KiCad/Freerouting mapping | [`UPSTREAM.md`](UPSTREAM.md) | Use this before selecting a native file; keep it current. |
| Requirements | [`autorouter-requirements.txt`](autorouter-requirements.txt) | Read it; do not rewrite it as part of a sync. |
| Regression cases | [`regression-corpus.yml`](regression-corpus.yml) | Run the relevant cases and record results. |

The `Reference commit` in `UPSTREAM.md` is the last accepted upstream baseline for this native
implementation. It is not permission to use a local clone. If a task is bootstrapping a new
baseline, the task must state the exact GitHub SHA explicitly and create the initial ledger from
that SHA.

## What “every PR” means

For a normal incremental synchronization, “every PR” means **every merged upstream PR whose
merged change is after the recorded synchronization baseline**, plus a recorded entry for every
open PR encountered during the inventory. A full-history audit can use the repository's first
usable baseline instead, but must say so before it starts.

The selection is not based only on dates or titles:

1. Enumerate all PRs from GitHub with pagination.
2. Resolve each candidate's immutable base, head, and merge commits.
3. Check ancestry against the recorded baseline in a temporary GitHub clone.
4. Include candidates whose ancestry cannot be proved in a `needs-review` set rather than silently
   dropping them. This matters for squash merges, rebases, deleted branches, and rewritten history.
5. Sort merged candidates by merge time, then by merge SHA/PR number for deterministic processing.
6. Process older changes before newer changes. If a later PR contains an earlier change, mark the
   duplicate as `superseded` and retain both PR links in the ledger.

Open PRs are inventory items, not accepted source. Do not port an open PR unless the user
explicitly asks for an experimental port; label that work `pending-open` or `experimental` and do
not advance the stable baseline.

## Phase 0: preflight and safety

Before fetching anything:

```sh
git status --short
git diff --check
test -f autorouter-docs/UPSTREAM.md
test -f autorouter-docs/autorouter-requirements.txt
```

If the target contains unrelated user changes, preserve them. Do not reset, clean, stash, or
overwrite them without explicit permission. Use a user-requested branch/worktree if isolation is
needed. Keep all temporary upstream clones and downloaded patches outside the target, preferably
under `/private/tmp`.

Read these files before making an algorithm change:

1. `autorouter-docs/autorouter-requirements.txt`
2. `autorouter-docs/UPSTREAM.md`
3. `autorouter-docs/README.md`
4. the target files named by the mapping table for the selected upstream files

The requirements file is authoritative for behavior. The mapping file is authoritative for the
current native equivalent, not for upstream history.

## Phase 1: obtain the upstream inventory from GitHub

Use an authenticated `gh` session when available so that pagination and rate limits are visible.
Unauthenticated public API requests are acceptable for a small read-only run, but a rate-limit or
permission error is a blocking condition, not a reason to fall back to the local checkout.

The official GitHub documentation for the endpoints is:

* [List pull requests](https://docs.github.com/en/rest/pulls/pulls#list-pull-requests)
* [Get a pull request](https://docs.github.com/en/rest/pulls/pulls#get-a-pull-request)
* [List pull request files](https://docs.github.com/en/rest/pulls/pulls#list-pull-request-files)
* [GitHub REST API pagination](https://docs.github.com/en/rest/using-the-rest-api/using-pagination-in-the-rest-api)

Resolve the default branch and current tip instead of assuming `master` or `main`:

```sh
UPSTREAM_REPO='freerouting/freerouting'
UPSTREAM_URL='https://github.com/freerouting/freerouting.git'

UPSTREAM_DEFAULT_BRANCH="$({
  gh api "repos/${UPSTREAM_REPO}" --jq '.default_branch'
} 2>/dev/null || curl -fsSL "https://api.github.com/repos/${UPSTREAM_REPO}" | python3 -c \
  'import json,sys; print(json.load(sys.stdin)["default_branch"])')"

UPSTREAM_TIP="$(git ls-remote "$UPSTREAM_URL" "refs/heads/${UPSTREAM_DEFAULT_BRANCH}" | awk '{print $1}')"
test -n "$UPSTREAM_TIP"
printf 'branch=%s\ntip=%s\n' "$UPSTREAM_DEFAULT_BRANCH" "$UPSTREAM_TIP"
```

Read the exact `Reference commit` from `UPSTREAM.md` into `SYNC_BASE`. Do not guess it from the
current GitHub tip, from a local clone, or from a release name. Verify that it is a 40-character
commit SHA and fetch that object into the temporary clone in the next phase.

Enumerate **all** PRs, not just search results or the first page. The REST list endpoint supports
`state=all`, and GitHub documents `100` as the maximum `per_page` value; `--paginate` is therefore
required:

```sh
SYNC_WORK="$(mktemp -d /private/tmp/freerouting-codex-sync.XXXXXX)"

gh api --paginate --slurp \
  -H 'Accept: application/vnd.github+json' \
  "repos/${UPSTREAM_REPO}/pulls?state=all&sort=created&direction=asc&per_page=100" \
  > "${SYNC_WORK}/pull-request-pages.json"
```

`gh api --paginate --slurp` produces an array of response pages. Flatten it before filtering;
never treat the first page as the complete history:

```python
import json
from pathlib import Path

pages = json.loads(Path("pull-request-pages.json").read_text())
pulls = [pull for page in pages for pull in page]
for pull in pulls:
    print(
        pull["number"],
        pull["state"],
        pull.get("merged_at"),
        pull["html_url"],
    )
```

For every PR that could be in the synchronization range, request the detail, file list, and (when
needed) commit list. Paginate file and commit lists too:

```sh
PR_NUMBER='1234'
gh api "repos/${UPSTREAM_REPO}/pulls/${PR_NUMBER}" \
  -H 'Accept: application/vnd.github+json' \
  > "${SYNC_WORK}/pr-${PR_NUMBER}.json"
gh api --paginate --slurp "repos/${UPSTREAM_REPO}/pulls/${PR_NUMBER}/files?per_page=100" \
  > "${SYNC_WORK}/pr-${PR_NUMBER}-files.json"
gh api --paginate --slurp "repos/${UPSTREAM_REPO}/pulls/${PR_NUMBER}/commits?per_page=100" \
  > "${SYNC_WORK}/pr-${PR_NUMBER}-commits.json"
```

The inventory must retain, at minimum:

* PR number, title, state, `merged_at`, and HTML URL;
* `base.ref`, `base.sha`, `head.ref`, `head.sha`, and `merge_commit_sha`;
* every changed path and its status (`added`, `modified`, `renamed`, or `removed`);
* the API response or a checksum of the response used for the run.

Do not use GitHub's search endpoint as the sole inventory source: a search result set is not the
complete PR ledger. If the API returns a rate-limit, pagination, or truncation error, stop with an
explicit `blocked` result and preserve the error details.

## Phase 2: materialize exact PR diffs without using the local reference

Create a disposable, blob-filtered clone directly from GitHub:

```sh
git clone --filter=blob:none --no-checkout --no-tags \
  "$UPSTREAM_URL" "$SYNC_WORK/freerouting"
UPSTREAM_CLONE="$SYNC_WORK/freerouting"

git -C "$UPSTREAM_CLONE" fetch --no-tags origin "$SYNC_BASE"
git -C "$UPSTREAM_CLONE" fetch --no-tags origin "$UPSTREAM_TIP"
git -C "$UPSTREAM_CLONE" fetch --no-tags origin \
  "refs/pull/${PR_NUMBER}/head:refs/remotes/upstream/pr/${PR_NUMBER}"
```

For each PR, fetch the exact `base.sha` and `head.sha` from the API. Prefer the exact Git diff
between the PR merge base and head:

```sh
BASE_SHA='the API value, not a placeholder in a real run'
HEAD_SHA='the API value, not a placeholder in a real run'
PR_MERGE_BASE="$(git -C "$UPSTREAM_CLONE" merge-base "$BASE_SHA" "$HEAD_SHA")"

git -C "$UPSTREAM_CLONE" diff --name-status \
  "$PR_MERGE_BASE" "$HEAD_SHA" > "$SYNC_WORK/pr-${PR_NUMBER}-paths.txt"
git -C "$UPSTREAM_CLONE" diff --binary \
  "$PR_MERGE_BASE" "$HEAD_SHA" > "$SYNC_WORK/pr-${PR_NUMBER}.patch"
```

Also retain the authoritative GitHub patch URL for auditability:

```sh
curl --fail --location --silent --show-error \
  -H 'Accept: application/vnd.github.v3.diff' \
  "https://github.com/${UPSTREAM_REPO}/pull/${PR_NUMBER}.diff" \
  > "$SYNC_WORK/pr-${PR_NUMBER}-github.patch"
```

If the PR head ref was deleted, use the API's immutable `head.sha` and the `.diff` URL. If neither
can be obtained, mark the PR `blocked`; never substitute a local Freerouting checkout or a moving
branch. Review the patch and the file list together. The GitHub PR page is useful for context,
but the recorded SHAs and patch are the reproducible inputs.

To select PRs after `SYNC_BASE`, verify ancestry in the temporary clone:

```sh
git -C "$UPSTREAM_CLONE" merge-base --is-ancestor "$SYNC_BASE" "$MERGE_SHA"
```

If this check is false because the merge SHA is unavailable, the PR was rebased/squashed in a way
that cannot be resolved, or the upstream history was rewritten, retain it in `needs-review` and
compare its merge date and exact patch manually. A false ancestry check is never permission to
silently omit a PR.

## Phase 3: classify paths and isolate autorouter changes

Classify the actual changed files and, for mixed files, the actual hunks. Do not classify from the
title, labels, or PR description alone. Use `UPSTREAM.md` for the exact native mapping.

### Include or review as engine input

These Freerouting areas are candidates for a native port when the patch changes routing behavior:

```text
src/main/java/app/freerouting/autoroute/**
src/main/java/app/freerouting/board/**
src/main/java/app/freerouting/geometry/**
src/main/java/app/freerouting/drc/**
src/main/java/app/freerouting/rules/**
src/main/java/app/freerouting/core/**
```

The broad support packages are **review-only**, not blanket-copy areas. Port a support change only
when the changed behavior is required by an autorouter path and the corresponding KiCad adapter or
native rule API can implement it without importing Freerouting's application layer.

Relevant upstream tests under `src/test/java/` are behavior specifications. Translate their
assertions into KiCad unit, QA, or corpus tests; do not copy Java test infrastructure into KiCad.

Use these existing mapping groups first:

* `autoroute/maze` -> `pcbnew/autorouter/maze`
* `autoroute/expansion` -> `pcbnew/autorouter/expansion`
* `autoroute/drill` -> `pcbnew/autorouter/drill`
* `autoroute/path` -> `pcbnew/autorouter/path`
* `autoroute/pipeline` -> `pcbnew/autorouter/pipeline`
* board/rule/geometry support -> the corresponding `pcbnew/autorouter/board`, `drc`, or mapped
  native KiCad API documented in `UPSTREAM.md`

Keep Freerouting-shaped filenames and class names where practical. If a KiCad ownership or
threading boundary requires a different file, update `UPSTREAM.md` with the reason rather than
silently losing the correspondence.

### Always exclude from the port

Do not import these areas into the native autorouter:

```text
src/main/java/app/freerouting/gui/**
src/main/java/app/freerouting/api/**
src/main/java/app/freerouting/management/**
src/main/java/app/freerouting/io/**
src/main/java/app/freerouting/analytics/**
src/main/java/app/freerouting/mcp/**
src_v19/**
docs/**
integrations/**
.github/**
```

Also exclude release files, website assets, translations, build scripts, IDE files, screenshots,
and formatting-only changes unless the user explicitly requests a separate KiCad maintenance
change. A GUI-only PR is still recorded as `skipped-gui-only`; it is not ignored.

For a mixed PR:

1. Port only engine files or engine hunks that have a clear entry in `UPSTREAM.md`.
2. Do not port GUI settings panels, Swing event flow, API controllers, Specctra readers/writers,
   or Freerouting board serialization.
3. Re-express a required setting in KiCad's existing native settings/dialog boundary only if the
   algorithm cannot work without it; record that as an intentional host adaptation.
4. If the engine behavior is inseparable from an excluded GUI/I/O change, mark the PR `blocked`
   and explain the missing native seam instead of guessing.

## Phase 4: port the algorithm safely

For each qualifying PR:

1. Read the complete upstream patch, surrounding source, and relevant tests at the pinned head.
2. Read the current KiCad counterpart and its callers before editing it.
3. Apply the smallest semantic change to the mapped C++ files. Manual porting is expected because
   KiCad owns the board and uses immutable worker snapshots, while Freerouting owns a Java board.
4. Preserve deterministic ordering, tie-breaks, layer transitions, clearance expansion, drill
   spacing, same-net sharing, plane behavior, rip-up rules, cancellation, and metric semantics.
5. Keep `wxWidgets`, `BOARD`, `BOARD_COMMIT`, views, dialogs, and editor notifications on KiCad's
   editor/main-thread side. The worker must continue to consume the immutable snapshot and emit a
   proposal/result rather than mutating the live board.
6. Keep geometry conversion in the adapter. Do not introduce Freerouting coordinate units or
   mutable Java-board assumptions into the maze/search classes.
7. Port upstream tests as focused KiCad tests or corpus cases. A test that depends on Java GUI,
   Specctra serialization, or Freerouting's service layer is not an autorouter parity test.
8. Update the mapping and intentional-divergence notes when a new file, renamed seam, or host
   adaptation is introduced.

Prefer one native commit per upstream PR when the user requests commits. If several upstream PRs
are inseparable, group only those PRs, preserve their chronological order, and list every upstream
URL and SHA in the native commit and target PR. Do not hide unrelated cleanup in a synchronization
commit.

Preserve applicable upstream GPLv3 attribution and retain the upstream PR URL in the commit/PR
metadata. The target code must continue to follow KiCad's repository licensing and contribution
rules.

## Phase 5: maintain the machine-readable PR ledger

Every sync run must create or update `autorouter-docs/freerouting-pr-ledger.yml` unless an existing
project ledger has been explicitly selected. The ledger is the completeness proof for “every PR”
and is the source for the target PR body. Use actual API values; never invent a title, SHA, metric,
or target PR URL.

Each selected or encountered PR gets exactly one entry with this shape:

```yaml
- upstream_pr: 1234
  upstream_url: https://github.com/freerouting/freerouting/pull/1234
  title: "exact title from GitHub"
  state: merged
  merged_at: "2026-01-02T03:04:05Z"
  base_sha: "40-character SHA"
  head_sha: "40-character SHA"
  merge_sha: "40-character SHA or null"
  source_paths:
    - src/main/java/app/freerouting/autoroute/maze/MazeSearchEngine.java
  excluded_paths:
    - src/main/java/app/freerouting/gui/BoardFrame.java
  status: ported # ported | skipped-gui-only | skipped-non-autorouter | superseded | blocked | pending-open | needs-review | experimental | pending-review
  target_paths:
    - pcbnew/autorouter/maze/MazeSearchEngine.cpp
  target_commit: null
  target_pr: null
  tests:
    - command: "..."
      result: passed
  notes: "Intentional host differences, conflicts, or why no port was needed."
```

Ledger requirements:

* include skipped, superseded, blocked, and open entries, not only successful ports;
* list both included and excluded paths for mixed PRs;
* keep one stable status per PR and explain status changes in `notes`;
* add target commit/PR links only after they exist;
* do not mark `ported` until the focused build/tests and parity checks pass;
* use the ledger to prove that no PR page was skipped during pagination.

If the user asks only for an analysis and not an implementation, the ledger may be created with
`status: pending-review` entries, but no code or baseline update should be implied.

## Phase 6: validation gates

Run validation from the KiCad target root. Use the configured build directory if it exists; do not
assume a release build is equivalent to the current source.

```sh
cmake --build build/autorouter --parallel

build/autorouter/qa/tests/pcbnew/qa_pcbnew \
  '--run_test=NativeAutorouter/*' \
  --report_level=no

python3 scripts/autorouter/run_native_corpus.py \
  --binary build/autorouter/qa/tools/autorouter_parity/qa_autorouter_parity \
  --output-dir build/autorouter/corpus
```

For a same-board upstream comparison, export a DSN from the native QA harness and run a
Freerouting build made from the exact GitHub commit in a disposable directory. Do not use the
local reference checkout as the comparison input:

```sh
build/autorouter/qa/tools/autorouter_parity/qa_autorouter_parity \
  --board path/to/board.kicad_pcb \
  --export-dsn build/autorouter/board.dsn

java -jar /path/to/pinned-freerouting.jar \
  -de build/autorouter/board.dsn \
  -do build/autorouter/board.ses \
  -mp 1 -mt 0 --gui.enabled=false

python3 scripts/autorouter/compare_results.py reference.json native.json
```

The reference JAR must identify the same upstream commit recorded in the ledger. If an exact
head cannot be built, record `reference-unavailable` and the reason; do not claim parity from a
different release or moving nightly build.

At minimum, compare and record:

* unrouted connection count and completion rate;
* newly introduced KiCad DRC/clearance violations, including hole and board-edge checks;
* trace length, via count, layer transitions, rip-ups, passes, and expanded-node counts;
* deterministic route ordering/tie-break behavior on bounded corpus slices;
* cancellation and rejected-proposal behavior where the changed code can affect them.

Use the full KiCad DRC result for safety. Do not treat a partial shortcut count as proof that a
route is clean. Use `--max-nets`, `--max-connections`, `--include-net`, or other existing bounded
corpus options for diagnosis, then run the relevant full case before marking the PR `ported`.

Finish every run with:

```sh
git diff --check
git status --short
```

Generated binaries, patches, logs, corpus output, and temporary clones must remain outside the
committed source or be ignored. The final status must show only the intended target changes and
the synchronization documentation/ledger.

## Phase 7: target PR traceability

When the user authorizes a target commit or PR, make the upstream relationship explicit. Use a
native commit subject such as:

```text
autorouter: port Freerouting PR #1234
```

and include trailers or the commit body:

```text
Upstream-PR: https://github.com/freerouting/freerouting/pull/1234
Upstream-Base: <exact base SHA>
Upstream-Head: <exact head SHA>
Upstream-Merge: <exact merge SHA or n/a>
```

For a grouped sync, include one `Upstream-PR` line for every ported or intentionally skipped
engine-relevant PR. The target PR description should contain:

1. a table linking every ledger entry to its upstream PR;
2. the upstream baseline/tip and exact PR SHAs;
3. included engine paths, excluded GUI/I/O paths, and native target paths;
4. intentional KiCad host differences and any unresolved limitations;
5. build, focused QA, corpus, DRC, and reference-parity results;
6. rollback information (the native commit/branch and the upstream baseline it replaced).

Use `Ref: freerouting/freerouting#1234` plus the full URL for cross-repository references. Do not
use `Fixes` for an upstream PR unless the target repository actually owns the issue being fixed.
If the user has not authorized pushing or PR creation, provide the prepared body and links without
performing those actions.

### Target PR body template

```markdown
## Upstream source

This change ports autorouter behavior from Freerouting without importing Freerouting GUI or
Specctra/API layers.

- Repository: https://github.com/freerouting/freerouting
- Synchronization base: `<base SHA>`
- Upstream tip inspected: `<tip SHA>`

## Upstream PRs

| Upstream PR | Base | Head | Status | KiCad paths |
|---|---|---|---|---|
| [Freerouting #1234](https://github.com/freerouting/freerouting/pull/1234) | `<sha>` | `<sha>` | ported | `pcbnew/autorouter/...` |

## Scope and divergences

- Ported only engine/autorouter behavior.
- Excluded GUI, API, management, Specctra I/O, and release changes.
- `<list intentional KiCad adapter/transaction/geometry differences>`

## Verification

- Build: `<command/result>`
- Focused QA: `<command/result>`
- Corpus/parity: `<command/result>`
- New DRC violations: `<count>`
- Ledger: `autorouter-docs/freerouting-pr-ledger.yml`
```

## Completion checklist

A synchronization task is complete only when all applicable boxes are true:

- [ ] GitHub was queried directly; no local Freerouting checkout supplied the patch.
- [ ] The upstream default branch, baseline, tip, and every relevant PR SHA were recorded.
- [ ] All paginated PR pages and file pages were consumed.
- [ ] Every PR in range has exactly one ledger entry and a non-silent status.
- [ ] GUI/API/I/O-only changes were recorded but not ported.
- [ ] Mixed PRs list both included engine paths and excluded paths.
- [ ] Native filenames/structure follow `UPSTREAM.md` where practical.
- [ ] Requirements and reference checkout were not modified.
- [ ] Focused native build/tests and relevant corpus/parity checks passed or are explicitly recorded
      as blocked.
- [ ] No new DRC/clearance violations were introduced by the port.
- [ ] `UPSTREAM.md` was advanced only after the preceding checks passed.
- [ ] If a target PR was authorized, its body links every upstream PR directly and includes exact
      base/head/merge SHAs.

## Recommended Codex task prompt

Use this prompt when starting an actual synchronization run:

> Read `autorouter-docs/FREEROUTING-CODEX-SYNC.md`, `autorouter-docs/UPSTREAM.md`, and
> `autorouter-docs/autorouter-requirements.txt`. Fetch Freerouting directly from GitHub, enumerate
> every PR after the recorded upstream baseline with complete pagination, and create/update the
> PR ledger. Inspect exact base/head/merge SHAs and changed hunks. Port only autorouter/engine
> behavior through the mappings in `UPSTREAM.md`; do not copy GUI, API, management, Specctra I/O,
> or release code, and do not touch the local Freerouting checkout. Preserve Freerouting-shaped
> filenames where practical, validate each port with the native build, focused QA, regression
> corpus, DRC, and pinned-reference comparison, then update `UPSTREAM.md` only if all PRs are
> accounted for. Do not commit, push, or open a target PR unless I explicitly authorize it. If
> anything cannot be proven from GitHub or cannot be validated, record it as blocked rather than
> guessing.
