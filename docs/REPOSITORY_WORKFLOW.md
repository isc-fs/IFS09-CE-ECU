# Repository Workflow

## Permanent Branches

This repository uses two long-lived branches:

- `main`: production-ready and validated code only.
- `dev`: integration branch for ongoing development.

Do not work directly on either of them.

## Working Branches

Create all new work from `dev`:

- `feat/<short-slug>` for new functionality.
- `fix/<short-slug>` for bug fixes.

Use a **descriptive slug** — `fix/cell-derate-above-ams-trip`, not `fix/7`. A
name that says what the branch does is readable in a PR list a year later; a
counter is not.

The tracking-issue bot in `branch-issue.yml` predates this and still tries to
parse a leading number out of the branch name, so a slug-only branch opens an
issue carrying a "number NaN" warning. That warning is expected noise, not
something to fix by renaming the branch.

## Tracking Work With Issues

Each working branch should have one associated GitHub issue.

- Example branch: `feat/can3-mission-state`
- Example issue title: `[feat/can3-mission-state] Add CAN broadcast for mission state`

Open issues represent active work. Closed issues keep the historical record
after the branch is merged and deleted.

## Automation

The workflow in `.github/workflows/branch-issue.yml` manages issue tracking for
`feat/*` and `fix/*` branches:

- opens a tracking issue when the branch is first pushed
- applies the correct label
- warns if the branch number is not the expected next one
- fills the issue description from the first pushed commit message

## Typical Flow

1. Update `dev`.
2. Create `feat/<short-slug>` or `fix/<short-slug>` from `dev`.
3. Push the branch to GitHub.
4. Commit your work with a clear message.
5. Open a pull request targeting `dev`.
6. After review and merge into `dev`, delete the branch.
7. Merge `dev` into `main` only after full validation.
