# Using the lotusim-developer skill

This repo ships a coding-agent skill at `.agents/skills/lotusim-developer/`
(`SKILL.md` + `references/`). `AGENTS.md` at the repo root is read by every major
agent with no setup; this skill adds a deeper, task-oriented map (it opens with
6 field-tested pitfalls).

`SKILL.md` follows the open agent-skill format (name/description frontmatter +
markdown), so it works with any harness that supports skills.

## Codex

Zero setup — Codex auto-discovers skills under `.agents/skills/` (repo scope).

## Claude Code

Claude Code discovers skills under `.claude/skills/`. Link it once:

```bash
# this repo only
mkdir -p .claude/skills
ln -s ../../.agents/skills/lotusim-developer .claude/skills/lotusim-developer
# or user-wide (all your repos)
mkdir -p ~/.claude/skills
ln -s "$PWD/.agents/skills/lotusim-developer" ~/.claude/skills/lotusim-developer
```

## Universal installer (optional)

```bash
npx skills@latest add naval-group/LOTUSim
```

Fans the skill out to the agents you select. Third-party tool — review before use.

## Other harnesses

They all read `AGENTS.md` at the repo root with no setup. For first-class skill
support, see your harness's own skills documentation.
