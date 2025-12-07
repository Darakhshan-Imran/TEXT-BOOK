---
id: 0002
title: Update Constitution with Difficulty-Level Quality Standards
stage: constitution
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "quality-standards", "difficulty-levels", "chapter-requirements"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
null
---

## Prompt

```text
# Constitution Prompt for Claude Code

I'm building a Physical AI & Humanoid Robotics textbook for Panaversity hackathon. This is an educational Docusaurus-based web book with 6 complete chapters (Parts I-II) plus 16 summary chapters (Parts III-VI).

## Project Context

**Deliverable**: Educational textbook (theoretical learning, no hardware required)

**Target Audience**: Students with intermediate Python knowledge, no robotics background

**Technology Stack**:
- Docusaurus (documentation framework)
- Markdown chapters
- Mermaid diagrams (built-in)
- Python code examples
- RAG chatbot integration

**Existing Project Assets**:

I have already created reusable intelligence for this project:

**4 Skills** (in `.claude/skills/`):
1. `chapter-structure.md` - Generates consistent chapter templates
2. `code-example.md` - Creates production-quality code with documentation
3. `diagram-generator.md` - Builds Mermaid diagrams
4. `exercise-generator.md` - Creates conceptual exercises with solutions

**4 Subagents** (in `.claude/subagents/`):
1. `orchestrator.md` - Manages workflow, coordinates all subagents, ensures quality
2. `content-writer.md` - Writes educational content, uses skills: chapter-structure, diagram-generator, exercise-generator
3. `code-architect.md` - Creates code examples, uses skill: code-example
4. `diagram-designer.md` - Creates diagrams, uses skill: diagram-generator

## Book Structure

**6 Complete Chapters** (Progressive Difficulty):

**Part I: Foundations**
- Chapter 1: The Dawn of Embodied Intelligence (🟢 Beginner - Level 1)
- Chapter 2: The Physical AI Ecosystem (🟢 Beginner - Level 2)

**Part II: ROS 2 Fundamentals**
- Chapter 3: ROS 2 Architecture Fundamentals (🟡 Beginner-Intermediate - Level 3)
- Chapter 4: Building with ROS 2 and Python (🟡 Intermediate - Level 4)
- Chapter 5: Describing Robots with URDF (🟡 Intermediate - Level 5)
- Chapter 6: Launch Files and System Integration (🟠 Intermediate-Advanced - Level 6)

**16 Summary Chapters** (500-word placeholders for Parts III-VI showing learning path)

## Create Constitution Covering

Please create a comprehensive project constitution that establishes governing principles for:

### 1. Educational Philosophy
- Explanation-first approach ("why" before "how")
- Progressive complexity (beginner → advanced across 6 chapters)
- Conceptual mastery over memorization
- Real-world applications for every concept
- Theory (40%) / Practice examples (60%) balance
- Use analogies from everyday life
- No hardware required for learning

### 2. Chapter Quality Standards by Difficulty Level

**🟢 Beginner Chapters (1-2):**
- Word count: 1,800-2,500 words
- Code examples: 4-6 (30-80 lines each, very simple)
- Diagrams: 3-5 (simple comparisons)
- Math: None to basic
- Exercises: 2-3 (conceptual only)

**🟡 Intermediate Chapters (3-5):**
- Word count: 2,200-3,200 words
- Code examples: 6-8 (80-150 lines each, moderate complexity)
- Diagrams: 5-7 (detailed systems)
- Math: Basic to moderate
- Exercises: 3-4 (code comprehension + design)

**🟠 Advanced Chapter (6):**
- Word count: 2,800-3,500 words
- Code examples: 7-8 (100-200 lines each, production-ready)
- Diagrams: 6-7 (complex integrations)
- Math: As needed
- Exercises: 4-5 (system design + debugging)

**Summary Chapters (7-22):**
- Word count: 500 words
- Structure: Learning objectives, key topics, prerequisites, why it matters, coming soon section
- No code examples or diagrams (placeholder only)

[...additional sections covering content writing, code examples, diagrams, exercises, subagent coordination, file organization, quality checklist, and constraints...]
```

## Response snapshot

Updated the project constitution from version 1.0.0 to 2.0.0 with the following major changes:

**Key Updates:**
1. **Added Project Overview section** with deliverable, target audience, tech stack, and scope
2. **Restructured Chapter Quality Standards** from uniform requirements to tiered difficulty levels:
   - 🟢 Beginner (Ch 1-2): 1,800-2,500 words, 4-6 code examples, 3-5 diagrams, 2-3 exercises
   - 🟡 Intermediate (Ch 3-5): 2,200-3,200 words, 6-8 code examples, 5-7 diagrams, 3-4 exercises
   - 🟠 Advanced (Ch 6): 2,800-3,500 words, 7-8 code examples, 6-7 diagrams, 4-5 exercises
   - 📘 Summary (Ch 7-22): ~500 words, no code/diagrams/exercises
3. **Added Summary Chapters specification** with required sections (Learning Objectives, Key Topics, Prerequisites, Why It Matters, Coming Soon)
4. **Updated all tables** with difficulty-specific requirements for code complexity, diagram types, and exercise types
5. **Added separate QA checklists** for complete chapters vs summary chapters
6. **Included Sync Impact Report** as HTML comment documenting all changes

**Version bump rationale**: MAJOR (2.0.0) due to backward-incompatible changes replacing uniform chapter standards with tiered difficulty requirements.

## Outcome

- ✅ Impact: Major constitution revision establishing difficulty-level quality standards for all 22 chapters
- 🧪 Tests: N/A (governance document)
- 📁 Files: 1 file updated: .specify/memory/constitution.md
- 🔁 Next prompts: Begin chapter creation using updated standards; orchestrator can now enforce tiered requirements
- 🧠 Reflection: Successfully transformed uniform standards into progressive difficulty tiers that match the educational philosophy of "beginner to advanced" progression

## Evaluation notes (flywheel)

- Failure modes observed: None - existing constitution provided solid foundation for enhancement
- Graders run and results (PASS/FAIL): PASS - all placeholders filled, version incremented correctly, tiered requirements clearly specified
- Prompt variant (if applicable): null
- Next experiment: Validate that subagents correctly interpret and apply difficulty-level requirements during chapter creation
