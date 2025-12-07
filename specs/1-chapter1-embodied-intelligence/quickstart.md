# Quickstart: Chapter 1 Implementation

**Feature**: `1-chapter1-embodied-intelligence`
**Date**: 2025-12-07

## Prerequisites

Before implementing Chapter 1, ensure:

1. **Docusaurus Setup**
   - Docusaurus 3.x installed in `docusaurus-course-book/`
   - Mermaid theme enabled (`@docusaurus/theme-mermaid`)
   - Development server runs without errors

2. **Directory Structure**
   ```bash
   mkdir -p docusaurus-course-book/docs/part-1-foundations
   ```

3. **Constitution Loaded**
   - Constitution v2.0.0 at `.specify/memory/constitution.md`
   - All subagents aware of beginner chapter requirements

---

## Quick Implementation Steps

### Step 1: Create Chapter File

```bash
touch docusaurus-course-book/docs/part-1-foundations/chapter-01-embodied-intelligence.md
```

### Step 2: Add Frontmatter

```yaml
---
sidebar_position: 1
title: "Chapter 1: The Dawn of Embodied Intelligence"
---
```

### Step 3: Copy Section Structure

```markdown
# The Dawn of Embodied Intelligence

> **Difficulty**: 🟢 Beginner (Level 1)
> **Reading Time**: 20-25 minutes
> **Prerequisites**: None

## Introduction
[300-350 words - Hook + context + preview]

## Digital AI vs Physical AI
[400-450 words + Diagram 1 + Code Example 1]

## Why Humanoid Robots?
[350-400 words + Diagram 2 + Code Example 2]

## Physical AI System Components
[400-450 words + Diagram 3 + Code Example 3]

## Implementation Perspective
[200-250 words]

## Common Pitfalls
[Table with 3 rows]

## Real-World Applications
[100-150 words]

## Conceptual Exercises
[2 exercises with collapsible solutions]

## Key Takeaways
[5 bullet points]

## Further Reading
[3-4 resources]

## Next Chapter Preview
[2-3 sentences]
```

### Step 4: Implement Components

**Order of Implementation:**
1. Structure (sections with placeholders)
2. Introduction text
3. Core concepts text (3 sections)
4. Diagrams (parallel: diagram-designer)
5. Code examples (parallel: code-architect)
6. Supporting sections
7. Exercises with solutions
8. Final review

---

## Key Commands

### Verify Docusaurus Build
```bash
cd docusaurus-course-book
npm run build
```

### Start Development Server
```bash
cd docusaurus-course-book
npm run start
```

### Validate Chapter File
```bash
# Check frontmatter
head -10 docs/part-1-foundations/chapter-01-embodied-intelligence.md

# Check word count (approximate)
wc -w docs/part-1-foundations/chapter-01-embodied-intelligence.md
```

---

## Subagent Workflow

```
┌─────────────────────────────────────────────────────────────┐
│                     ORCHESTRATOR                             │
│  1. Create structure → Validate                              │
│  2. Request content → Review                                 │
│  3. Request code/diagrams → Validate                         │
│  4. Integrate → Final QA                                     │
└─────────────────────────────────────────────────────────────┘
        │                    │                    │
        ▼                    ▼                    ▼
┌───────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ CONTENT-WRITER│  │  CODE-ARCHITECT │  │ DIAGRAM-DESIGNER│
│ - Introduction│  │ - Example 1     │  │ - Diagram 1     │
│ - Core text   │  │ - Example 2     │  │ - Diagram 2     │
│ - Exercises   │  │ - Example 3     │  │ - Diagram 3     │
│ - Takeaways   │  │                 │  │ - Diagram 4?    │
└───────────────┘  └─────────────────┘  └─────────────────┘
```

---

## Quality Checkpoints

### After Structure (Phase 1)
- [ ] All 11 sections present
- [ ] Frontmatter correct
- [ ] File in correct location

### After Content (Phase 2)
- [ ] Word count: 1,800-2,500
- [ ] Active voice throughout
- [ ] All terms defined
- [ ] No jargon without explanation

### After Code/Diagrams (Phase 3)
- [ ] 3 code examples complete
- [ ] 3-4 diagrams render
- [ ] Colors match constitution
- [ ] Context/caption/explanation for each

### After Exercises (Phase 4)
- [ ] 2 exercises with solutions
- [ ] Solutions are collapsible
- [ ] Detailed explanations included

### Final QA (Phase 5)
- [ ] Docusaurus build passes
- [ ] All diagrams render correctly
- [ ] Mobile view readable
- [ ] Learning objectives covered

---

## Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| Mermaid not rendering | Check `@docusaurus/theme-mermaid` in config |
| Colors not showing | Use `classDef` with fill property |
| Code not highlighted | Ensure ` ```python ` syntax |
| Build fails | Check for unclosed tags, invalid YAML |
| Word count too low | Expand explanations, add analogies |
| Word count too high | Remove redundancy, tighten sentences |
