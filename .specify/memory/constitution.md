<!--
SYNC IMPACT REPORT
==================
Version Change: 1.0.0 → 2.0.0
Bump Rationale: MAJOR - Significant restructuring of quality standards with
                backward-incompatible changes to chapter requirements (tiered
                difficulty levels replace uniform standards)

Modified Sections:
- "II. Content Quality Standards" → "II. Chapter Quality Standards by Difficulty Level"
- "III. Code Example Standards" → Updated with difficulty-specific quantities
- "IV. Visual Content Standards" → Updated with difficulty-specific quantities
- "V. Exercise Standards" → Updated with difficulty-specific quantities

Added Sections:
- Summary Chapters (7-22) specification with 500-word placeholder format
- Detailed difficulty level indicators (Beginner/Intermediate/Advanced)
- Chapter-specific word count, code, diagram, and exercise requirements

Removed/Replaced:
- Uniform 2,500-4,000 word requirement (replaced with tiered requirements)
- Single 8-12 code examples requirement (replaced with tiered requirements)
- Single 5-7 diagrams requirement (replaced with tiered requirements)

Templates Requiring Updates:
- ✅ Skills: chapter-structure.md (will dynamically reference constitution)
- ✅ Skills: code-example.md (will dynamically reference constitution)
- ✅ Skills: diagram-generator.md (will dynamically reference constitution)
- ✅ Skills: exercise-generator.md (will dynamically reference constitution)
- ✅ Agents: All agents reference constitution dynamically

Deferred Items: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Project Overview

**Deliverable**: Educational textbook for the Panaversity Hackathon (theoretical learning, no hardware required)

**Target Audience**: Students with intermediate Python knowledge, no robotics background

**Technology Stack**: Docusaurus, Markdown chapters, Mermaid diagrams (built-in), Python code examples, RAG chatbot integration

**Scope**: 6 complete chapters (Parts I-II) + 16 summary chapters (Parts III-VI)

---

## Core Principles

### I. Educational Philosophy

**Guiding Principles:**
- **Explanation-First**: Always explain "why" before "how". This ensures foundational understanding before practical application.
- **Progressive Complexity**: Concepts are introduced from simple (Level 1) to advanced (Level 6), building knowledge incrementally across chapters.
- **Conceptual Mastery**: Focus is on understanding underlying principles rather than rote memorization of commands or syntax.
- **Real-World Relevance**: Every concept and example MUST be tied to industry applications, demonstrating practical utility.
- **Accessibility**: All learning is theoretical; no hardware or complex setup is required, making the content accessible to a broader audience.

**Learning Approach:**
- Theory (40%) / Practice Examples (60%) balance to reinforce understanding.
- Multiple mental models for the same concept to cater to diverse learning styles.
- Analogies from everyday life to simplify complex ideas.
- Visual-first explanations to enhance comprehension.
- No hardware required for any learning activity.

---

### II. Chapter Quality Standards by Difficulty Level

**Book Structure:**

| Part | Chapters | Difficulty | Status |
|------|----------|------------|--------|
| Part I: Foundations | 1-2 | 🟢 Beginner (Level 1-2) | Complete |
| Part II: ROS 2 Fundamentals | 3-6 | 🟡 Intermediate to 🟠 Advanced (Level 3-6) | Complete |
| Part III-VI: Advanced Topics | 7-22 | Various | Summary Only |

---

#### 🟢 Beginner Chapters (Chapters 1-2)

**Chapter 1: The Dawn of Embodied Intelligence** (Level 1)
**Chapter 2: The Physical AI Ecosystem** (Level 2)

| Metric | Requirement |
|--------|-------------|
| Word Count | 1,800 - 2,500 words |
| Code Examples | 4-6 examples (30-80 lines each, very simple) |
| Diagrams | 3-5 diagrams (simple comparisons) |
| Math | None to basic |
| Exercises | 2-3 (conceptual only) |

**Content Focus:**
- Foundation building with accessible language
- Heavy use of analogies from everyday life
- Minimal technical prerequisites
- Emphasis on "why Physical AI matters"

---

#### 🟡 Intermediate Chapters (Chapters 3-5)

**Chapter 3: ROS 2 Architecture Fundamentals** (Level 3 - Beginner-Intermediate)
**Chapter 4: Building with ROS 2 and Python** (Level 4 - Intermediate)
**Chapter 5: Describing Robots with URDF** (Level 5 - Intermediate)

| Metric | Requirement |
|--------|-------------|
| Word Count | 2,200 - 3,200 words |
| Code Examples | 6-8 examples (80-150 lines each, moderate complexity) |
| Diagrams | 5-7 diagrams (detailed systems) |
| Math | Basic to moderate |
| Exercises | 3-4 (code comprehension + design) |

**Content Focus:**
- Building on foundational knowledge
- Introduction of ROS 2 concepts and patterns
- Practical code examples with clear explanations
- Bridge between theory and application

---

#### 🟠 Advanced Chapter (Chapter 6)

**Chapter 6: Launch Files and System Integration** (Level 6 - Intermediate-Advanced)

| Metric | Requirement |
|--------|-------------|
| Word Count | 2,800 - 3,500 words |
| Code Examples | 7-8 examples (100-200 lines each, production-ready) |
| Diagrams | 6-7 diagrams (complex integrations) |
| Math | As needed |
| Exercises | 4-5 (system design + debugging) |

**Content Focus:**
- Integration of all previous concepts
- Production-ready patterns and best practices
- Complex system interactions
- Preparation for real-world development

---

#### 📘 Summary Chapters (Chapters 7-22)

**Purpose**: Placeholder chapters showing the complete learning path for Parts III-VI

| Metric | Requirement |
|--------|-------------|
| Word Count | ~500 words |
| Code Examples | None |
| Diagrams | None |
| Exercises | None |

**Required Sections:**
1. **Learning Objectives** - What students will learn (bullet list)
2. **Key Topics** - Main topics covered (brief descriptions)
3. **Prerequisites** - What students need to know first
4. **Why It Matters** - Real-world relevance and applications
5. **Coming Soon** - Encouragement and roadmap indication

---

### III. Content Writing Standards

**Tone**: Professional but approachable, adopting the voice of a knowledgeable mentor.

**Voice**: Active voice, present tense, second person ("you will learn") for direct engagement.

**Clarity**: Beginner-friendly language without oversimplification.

**Sentence Structure:**
- Average sentence length: 15-20 words
- Maximum sentence length: 30 words
- Vary sentence structure for readability

**Technical Terms:**
- Define immediately upon first use
- Provide pronunciation guide for complex terms where helpful
- Build a consistent glossary across chapters

**MUST Avoid:**
- Passive voice (except where absolutely necessary)
- Unexplained jargon
- Unnecessarily complex language
- Assumptions about prior robotics knowledge

**Chapter Template Structure (All Chapters MUST Follow):**
1. Frontmatter (sidebar_position, title)
2. Overview/Introduction
3. Learning Objectives
4. Core Concepts (with diagrams)
5. Implementation Guide (with code examples)
6. Common Pitfalls
7. Real-World Applications
8. Conceptual Exercises
9. Key Takeaways
10. Further Reading
11. Next Chapter Preview

---

### IV. Code Example Standards

**Completeness:**
- All code examples MUST be complete, self-contained, and theoretically runnable.
- No snippets or partial code examples are allowed.
- Every Python example MUST include:
  - All necessary imports
  - Main function
  - `if __name__ == "__main__":` guard
- Include clear example usage and expected output.

**Documentation Requirements:**

```python
"""
Module-level docstring explaining the overall purpose.
"""

class ExampleClass:
    """
    Class docstring with attributes and usage examples.

    Attributes:
        attribute_name: Description of the attribute.
    """

    def example_method(self, param: str) -> bool:
        """
        Brief description of what the method does.

        Args:
            param: Description of the parameter.

        Returns:
            Description of what is returned.

        Raises:
            ValueError: When the param is invalid.
        """
        pass
```

**Code Standards:**
- **Python**: PEP 8 compliance, comprehensive type hints, robust error handling
- **ROS 2**: Strict adherence to ROS 2 conventions, proper node lifecycle management
- **XML/URDF**: Well-commented robot descriptions with clear structure
- **YAML**: Clear, logical structure with inline explanations

**Complexity by Chapter Level:**

| Level | Lines per Example | Complexity |
|-------|-------------------|------------|
| 🟢 Beginner | 30-80 lines | Single file, simple logic, minimal dependencies |
| 🟡 Intermediate | 80-150 lines | Multiple classes, good error handling |
| 🟠 Advanced | 100-200 lines | Production features, full logging |

---

### V. Diagram Standards

**All diagrams MUST use Mermaid syntax** (Docusaurus built-in support).

**Design Principles:**
- Maximum one core concept per diagram
- Consistent left-to-right or top-to-bottom flow
- Mobile-friendly (avoid excessive width)
- Clear, concise labels on all elements

**Required Color Scheme:**

| Color | Hex Code | Usage |
|-------|----------|-------|
| Blue | #e3f2fd | Hardware/Physical components |
| Orange | #fff3e0 | Middleware/Communication |
| Purple | #f3e5f5 | Application/Software |
| Green | #e1f5e1 | Success/Start states |
| Red | #ffe1e1 | Error/End states |
| Yellow | #fff4e1 | Warning/Decision points |

**Diagram Types:**
- **Flowchart**: Algorithms, decision trees, processes
- **Sequence**: Node interactions, message passing, time-based events
- **Architecture**: System components, layers, modules
- **State**: Robot modes, transitions, lifecycle
- **Class**: Software structure, inheritance, relationships

**Context Requirements:**
Every diagram MUST have:
1. Context-setting paragraph (before the diagram)
2. Clear caption (after the diagram)
3. Explanatory paragraph (after the diagram)

**Quantity by Chapter Level:**

| Level | Diagrams Required |
|-------|-------------------|
| 🟢 Beginner | 3-5 (simple comparisons) |
| 🟡 Intermediate | 5-7 (detailed systems) |
| 🟠 Advanced | 6-7 (complex integrations) |
| 📘 Summary | 0 |

---

### VI. Exercise Standards

**Exercise Types:**
- **Analysis Exercise**: Tests conceptual understanding through scenario-based questions
- **Design Exercise**: Requires applying knowledge to design a system component
- **Debugging Exercise**: Focuses on problem-solving by identifying and fixing issues
- **Comparison Exercise**: Encourages critical thinking by comparing approaches

**Exercise Quality Requirements:**
- MUST directly test concepts introduced in the chapter
- Difficulty level MUST match chapter level
- Complete solutions MUST be provided with detailed explanations
- No hardware or simulation required
- Real-world relevance required

**Solution Format:**
1. Step-by-step explanation
2. Key concepts highlighted
3. Common mistakes addressed
4. Clear rationale for why the solution works

**Quantity by Chapter Level:**

| Level | Exercises Required | Types |
|-------|-------------------|-------|
| 🟢 Beginner | 2-3 | Conceptual only |
| 🟡 Intermediate | 3-4 | Code comprehension + design |
| 🟠 Advanced | 4-5 | System design + debugging |
| 📘 Summary | 0 | N/A |

---

### VII. Technical Implementation Standards

**Docusaurus Integration:**
- Chapters saved as: `docs/part-X-name/chapter-XX-title-slug.md`
- Required Frontmatter: `sidebar_position` and `title`
- Mermaid diagrams MUST render correctly
- Code syntax highlighting required for: Python, YAML, XML, Bash
- All internal links MUST use relative paths

**File Organization:**
```
docs/
├── part-1-foundations/
│   ├── chapter-01-embodied-intelligence.md
│   └── chapter-02-physical-ai-ecosystem.md
├── part-2-ros2-fundamentals/
│   ├── chapter-03-ros2-architecture.md
│   ├── chapter-04-ros2-python.md
│   ├── chapter-05-urdf.md
│   └── chapter-06-launch-files.md
├── part-3-[name]/
│   └── chapter-07-[title].md (summary)
│   ... (summary chapters 7-22)
static/
└── img/
    └── part-X/
```

**Naming Conventions:**
- Folder names: `part-X-kebab-case-name`
- File names: `chapter-XX-kebab-case-title.md`
- Image files: `chapter-XX-descriptive-name.png`

**Performance Requirements:**
- Chapter load time: Under 2 seconds
- All images: Optimized for web
- Mermaid diagrams: Optimized for rendering speed

---

### VIII. Subagent Coordination

**Subagent Roles:**

| Agent | Responsibility | Skills Used |
|-------|---------------|-------------|
| **Orchestrator** | Manages workflow, coordinates subagents, quality control | All |
| **Content-Writer** | Writes explanations, analogies, exercises | chapter-structure, diagram-generator, exercise-generator |
| **Code-Architect** | Creates production-quality code examples | code-example |
| **Diagram-Designer** | Designs Mermaid diagrams with consistent styling | diagram-generator |

**Collaboration Standards:**
- The Orchestrator MUST validate output from all other subagents
- The Content-Writer MUST request diagrams from Diagram-Designer (NOT create directly)
- The Content-Writer MUST request code from Code-Architect (NOT create directly)
- All subagents MUST strictly adhere to this constitution
- Quality checkpoints are REQUIRED after each phase

**Workflow:**
```
1. Planning     → Orchestrator defines requirements
2. Structure    → Content-Writer creates outline
3. Content      → Content-Writer drafts text
4. Code         → Code-Architect creates examples
5. Diagrams     → Diagram-Designer creates visuals
6. Exercises    → Content-Writer + Orchestrator
7. Integration  → Orchestrator merges all components
8. QA           → Orchestrator validates against checklist
9. Delivery     → Final chapter output
```

---

### IX. Quality Assurance

**Pre-Completion Checklist (Complete Chapters 1-6):**

- [ ] Meets word count for difficulty level
- [ ] Required number of code examples (all complete and documented)
- [ ] Required number of diagrams (all render correctly)
- [ ] Required number of exercises with complete solutions
- [ ] All template sections present
- [ ] Frontmatter correct (sidebar_position, title)
- [ ] No spelling or grammar errors
- [ ] Mobile-responsive design confirmed
- [ ] All internal links functional
- [ ] Follows all constitution standards
- [ ] Learning objectives explicitly addressed

**Pre-Completion Checklist (Summary Chapters 7-22):**

- [ ] Word count approximately 500 words
- [ ] All required sections present (Learning Objectives, Key Topics, Prerequisites, Why It Matters, Coming Soon)
- [ ] Frontmatter correct
- [ ] No placeholder text or TODO markers
- [ ] Consistent tone with complete chapters

**Technical Validation:**
- Chapter loads in Docusaurus without errors
- All Mermaid diagrams render correctly
- All code blocks have appropriate syntax highlighting
- Navigation works correctly
- Mobile view is readable and well-formatted

---

### X. Constraints & Boundaries

**What We DON'T Do:**
- ❌ No hands-on hardware labs (pure theoretical learning)
- ❌ No actual robot simulation required
- ❌ No installation guides for software
- ❌ No platform-specific setup instructions

**What We DO:**
- ✅ Provide complete code examples (theoretically runnable)
- ✅ Explain concepts thoroughly with diagrams
- ✅ Show real-world applications and industry examples
- ✅ Prepare students with foundational knowledge for future hands-on work

**Project Scope:**
- 22 chapters total across 6 parts
- 6 complete chapters (Parts I-II)
- 16 summary chapters (Parts III-VI)
- Primary focus: Physical AI, ROS 2, URDF, Launch Files
- Future topics: Gazebo, NVIDIA Isaac, Vision-Language-Action, Humanoid Robotics

---

### XI. Terminology & Consistency

**Standard Terms:**
- Always "ROS 2" (not "ROS2" or "ros2")
- Always "Physical AI" (not "physical AI" or "PhysicalAI")
- Always "URDF" (all caps)
- Always "Python" (capitalized)

**Glossary Management:**
- Define each technical term upon first use
- Maintain consistent definitions across chapters
- Link to glossary for complex terms

**Cross-References:**
- Link to prerequisite chapters when building on previous knowledge
- Include forward references when introducing foundational concepts
- All links MUST be valid and functional

---

## Governance

This constitution serves as the foundational governance document for the Physical AI & Humanoid Robotics Textbook project, superseding all other conflicting practices or guidelines.

**Amendment Procedure**: Any amendments to this constitution require:
1. Documented proposal with rationale
2. Approval by project stakeholders
3. Clear migration plan for affected content
4. Version bump following semantic versioning

**Versioning Policy**: The constitution version adheres to semantic versioning (MAJOR.MINOR.PATCH):
- **MAJOR**: Backward incompatible changes, principle removals, or significant redefinitions
- **MINOR**: Addition of new principles, sections, or materially expanded guidance
- **PATCH**: Clarifications, wording refinements, typo fixes, or non-semantic updates

**Compliance Review**: All pull requests, code reviews, and content audits MUST verify compliance with the principles and standards outlined herein.

---

**Version**: 2.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
