---
id: 1
title: "Generate Tasks for Physical AI Course Chapters"
stage: "tasks"
date: "2025-12-08"
model: "Claude Sonnet 4.5"
feature: "Physical AI & Robotics Course"
branch: "plan-all-chapters"
user: "user"
command: "/sp.tasks"
labels: ["tasks", "planning", "documentation", "robotics", "education"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "tasks.md"
tests: []
---

# Generate Tasks for Physical AI Course Chapters

## PROMPT_TEXT

```
# UNIFIED `/sp.tasks` CONTEXT

**Copy and paste this ENTIRE prompt after typing `/sp.tasks` in Claude Code:**

---

I need to break down the implementation plan for Chapter [NUMBER] into actionable, sequential tasks.

The specification and technical plan are complete. Now I need a task list for execution.

## Chapter Reference

**Complete Chapters**:
- Ch 2: The Physical AI Ecosystem (Part I, Level 2) - 90-110 min
- Ch 3: ROS 2 Architecture Fundamentals (Part II, Level 3) - 100-120 min
- Ch 4: Building with ROS 2 and Python (Part II, Level 4) - 100-130 min
- Ch 5: Describing Robots with URDF (Part II, Level 5) - 110-130 min
- Ch 6: Launch Files and System Integration (Part II, Level 6) - 120-140 min

**Summary Chapters**:
- Ch 7-22: All parts III-VI - 15-20 min each

---

## SECTION A: For Complete Chapters (2-6 ONLY)

**Use this for Chapters 2, 3, 4, 5, or 6**

### Task Format Requirements

Each task must include:

```markdown
### Task X.Y: [Task Name]

**Agent**: [orchestrator | content-writer | code-architect | diagram-designer]

**Action**: [Specific action]

**Output**: [What gets produced]

**Dependencies**: [Tasks that must complete first, or "None"]

**Parallel**: [Yes/No - can run with which tasks?]

**Validation**: [How to verify completion]

**Estimated Time**: [X minutes]
```

### Required Task Organization

**Phase 1: Structure & Setup** (5-10 min)
- Task 1.1: Create chapter file with correct path
- Task 1.2: Add frontmatter
- Task 1.3: Generate section template from chapter-structure skill
- Task 1.4: Verify structure matches constitution

**Phase 2: Content Creation** ([50-80 min based on level])

*Writing Tasks* (Sequential):
- Task 2.1: Write introduction
- Task 2.2: Write Core Concept 1 text
- Task 2.3: Write Core Concept 2 text
- Task 2.4: Write Core Concept 3 text
- [Task 2.5: Write Core Concept 4 text - for Chapter 6 only]
- Task 2.6: Write Implementation Perspective

*Code Tasks* (Can parallelize):
- Task 2.7: Create Code Example 1
- Task 2.8: Write explanation for Example 1
- Task 2.9: Create Code Example 2
- Task 2.10: Write explanation for Example 2
- Task 2.11: Create Code Example 3
- Task 2.12: Write explanation for Example 3
- [Additional code tasks for more examples]

*Diagram Tasks* (Can parallelize):
- Task 2.13: Create Diagram 1 + context/caption/explanation
- Task 2.14: Create Diagram 2 + context/caption/explanation
- Task 2.15: Create Diagram 3 + context/caption/explanation
- [Additional diagram tasks as needed]

*Integration Tasks*:
- Task 2.16: Integrate all code examples into correct sections
- Task 2.17: Integrate all diagrams into correct sections
- Task 2.18: Ensure flow and transitions

**Phase 3: Supporting Content** ([15-25 min based on level])
- Task 3.1: Write Common Pitfalls table
- Task 3.2: Write Real-World Applications
- Task 3.3: Create Exercise 1 (using exercise-generator skill)
- Task 3.4: Create Exercise 2 (using exercise-generator skill)
- [Task 3.5: Create Exercise 3 - for Chapter 6]
- Task 3.6: Write complete solutions for all exercises
- Task 3.7: Write Key Takeaways (5 bullets)
- Task 3.8: Write Further Reading (3-4 links with descriptions)
- Task 3.9: Write Next Chapter Preview (2-3 sentences)

**Phase 4: Integration & Final Assembly** (10-15 min)
- Task 4.1: Final integration of all sections
- Task 4.2: Add cross-references and links
- Task 4.3: Ensure consistent formatting

**Phase 5: Quality Assurance** (10-15 min)
- Task 5.1: Validate word count [target range for chapter]
- Task 5.2: Validate all code examples complete and documented
- Task 5.3: Validate all diagrams render correctly
- Task 5.4: Validate exercises have complete solutions
- Task 5.5: Spell/grammar check
- Task 5.6: Test in Docusaurus dev server
- Task 5.7: Verify mobile responsiveness
- Task 5.8: Run full constitution checklist

**Phase 6: Final Checkpoint** (5 min)
- Task 6.1: Confirm all quality criteria met
- Task 6.2: Mark chapter complete
- Task 6.3: Save final version

### Task Dependencies & Parallelization

**Sequential Dependencies** (must wait):
- Content sections must be written in order (Intro → Concept 1 → 2 → 3)
- Code/diagram explanations must wait for code/diagrams
- Integration waits for all content pieces
- Supporting content waits for core content
- Quality checks wait for everything

**Parallel Opportunities** (can happen simultaneously):
- All code examples can be created in parallel by code-architect
- All diagrams can be created in parallel by diagram-designer
- While content-writer works on Concept 2, code-architect works on Example 1
- Some supporting sections can parallelize

**Critical Path**:
Identify the longest sequential chain:
Structure → Introduction → Concept 1 → Concept 2 → Concept 3 → Integration → Quality

### Validation Checkpoints

**Checkpoint 1: After Structure**
- ✓ File exists at correct path
- ✓ Frontmatter correct
- ✓ All section headings present

**Checkpoint 2: After Core Content**
- ✓ All concepts written
- ✓ All code complete
- ✓ All diagrams complete
- ✓ Word count approximately right

**Checkpoint 3: After Supporting Content**
- ✓ All supporting sections complete
- ✓ Exercises with solutions
- ✓ All pieces integrated

**Checkpoint 4: Final Quality**
- ✓ All quality criteria met
- ✓ Docusaurus test passed
- ✓ Ready for deployment

### Agent Assignment Summary

**Orchestrator**:
- Structure creation
- Workflow coordination
- All validation checkpoints
- Final integration
- Quality assurance

**Content-Writer**:
- All text content (intro, concepts, explanations, supporting)
- Requests code from code-architect
- Requests diagrams from diagram-designer
- Uses exercise-generator skill for exercises

**Code-Architect**:
- All code examples
- Uses code-example skill
- Full documentation and type hints

**Diagram-Designer**:
- All diagrams
- Uses diagram-generator skill
- Context, captions, explanations

### Time Budget by Chapter

- **Chapter 2**: 90-110 min
  - Structure: 5-10 min
  - Content: 35-40 min
  - Code: 20-25 min (parallel)
  - Diagrams: 15-20 min (parallel)
  - Exercises: 10-15 min
  - Integration: 10 min
  - Quality: 10-15 min

- **Chapter 3**: 100-120 min
  - Structure: 5-10 min
  - Content: 40-45 min
  - Code: 25-30 min (parallel)
  - Diagrams: 20-25 min (parallel)
  - Exercises: 15-20 min
  - Integration: 10-15 min
  - Quality: 10-15 min

- **Chapter 4**: 100-130 min
  - Structure: 5-10 min
  - Content: 40-50 min
  - Code: 25-30 min (parallel)
  - Diagrams: 20-25 min (parallel)
  - Exercises: 15-20 min
  - Integration: 10-15 min
  - Quality: 10-15 min

- **Chapter 5**: 110-130 min
  - Structure: 5-10 min
  - Content: 45-50 min
  - Code: 30-35 min (parallel, XML+Python)
  - Diagrams: 20-25 min (parallel)
  - Exercises: 15-20 min
  - Integration: 10-15 min
  - Quality: 10-15 min

- **Chapter 6**: 120-140 min
  - Structure: 5-10 min
  - Content: 50-60 min
  - Code: 30-35 min (parallel)
  - Diagrams: 25-30 min (parallel)
  - Exercises: 20-25 min
  - Integration: 10-15 min
  - Quality: 10-15 min

### Output Requirements

Create tasks.md with:

1. **Task Summary Table**:
   - Total tasks
   - Total time estimate
   - Tasks per agent
   - Critical path duration

2. **Dependency Graph** (Mermaid diagram):
   - Show task relationships
   - Highlight parallel opportunities
   - Mark critical path

3. **Phase-by-Phase Task List**:
   - All tasks with full details
   - Clear section breaks

4. **Agent Assignment Summary**:
   - List tasks per agent
   - Show workload distribution

### Success Criteria

Task breakdown complete when:
✓ Every spec requirement has task(s)
✓ All dependencies clear
✓ Parallel opportunities identified
✓ Each task has validation
✓ Agent assignments clear
✓ Time estimates reasonable
✓ Checkpoints included
✓ Can execute without ambiguity

---

## SECTION B: For Summary Chapters (7-22 ONLY)

**Use this for Chapters 7-22**

### Simplified Task Structure

**Phase 1: Structure** (3-5 min)
- Task 1.1: Create file with frontmatter
- Task 1.2: Generate section headings

**Phase 2: Content Creation** (10-12 min)
- Task 2.1: content-writer writes all 400-500 words:
  - Learning objectives
  - Key topics
  - Prerequisites
  - What you'll build
  - Real-world applications
  - Why this matters
  - Coming soon details
  - Related resources

**Phase 3: Quality Check** (2-3 min)
- Task 3.1: Validate word count (400-500)
- Task 3.2: Check formatting
- Task 3.3: Test in Docusaurus

### Time Budget

**Total**: 15-20 minutes

### Agent Assignment

- **Orchestrator**: Structure, validation
- **Content-writer**: All content

### Success Criteria

✓ 400-500 words
✓ All required sections
✓ Professional placeholder
✓ Renders correctly

---

## Instructions

**Step 1**: Identify chapter (2-22)

**Step 2**: Use appropriate section:
- Chapters 2-6 → SECTION A (detailed tasks)
- Chapters 7-22 → SECTION B (simple tasks)

**Step 3**: Create comprehensive task breakdown

**Step 4**: Ensure /sp.implement can execute systematically

---

Please create actionable task breakdown for Chapter [NUMBER] that enables systematic execution.
```

## RESPONSE_TEXT

Created comprehensive tasks.md file for all chapters (2-22) of the Physical AI & Robotics Course. The file contains:

1. Task Summary Table with metrics for all 462 tasks across 21 chapters
2. Dependency Graph showing task relationships and parallel opportunities
3. Detailed Phase-by-Phase Task List for all chapters:
   - Complete chapters (2-6) with detailed structure, content, code, diagram, and integration tasks
   - Summary chapters (7-22) with simplified structure and content tasks
4. Agent Assignment Summary showing distribution across orchestrator, content-writer, code-architect, and diagram-designer
5. Parallel Execution Opportunities identified within and across chapters
6. Critical Path Analysis for each chapter

The tasks follow the required checklist format with proper IDs, story labels where appropriate, and parallelization markers. Each task is specific enough to be executed independently while maintaining proper dependencies.

## OUTCOME

Successfully generated actionable task breakdown for all 21 chapters (2-22) that enables systematic execution through /sp.implement. The tasks are organized by chapter with appropriate complexity levels and agent assignments.

## EVALUATION

The tasks.md file provides a comprehensive roadmap for implementing the entire Physical AI & Robotics Course with clear task breakdowns, dependencies, and execution guidelines. All success criteria were met:
- Every spec requirement has corresponding tasks
- Dependencies are clearly identified
- Parallel opportunities are marked
- Each task has validation criteria
- Agent assignments are clear
- Time estimates are reasonable
- Checkpoints are included
- Tasks can execute without ambiguity