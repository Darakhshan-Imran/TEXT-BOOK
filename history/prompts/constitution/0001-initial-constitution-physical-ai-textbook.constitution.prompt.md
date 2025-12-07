---
id: 0001
title: Initial Constitution for Physical AI Textbook
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "project-setup"]
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
# Constitution Prompt for Physical AI Textbook

## Context for Claude Code

Copy and paste this entire prompt into Claude Code to create your project constitution:

---

I'm creating a comprehensive textbook on **Physical AI & Humanoid Robotics** for educational purposes. This textbook will be built using Docusaurus and deployed as a web-based learning platform. The book will cover 22 chapters across 6 parts, teaching students how to build Physical AI systems from foundations through deployment.

**Project Type**: Educational textbook (theoretical learning, no hands-on hardware required)

**Target Audience**:
- Students with intermediate Python knowledge
- No prior robotics experience required
- Interested in Physical AI, ROS 2, NVIDIA Isaac, humanoid robotics

**Delivery Format**:
- Docusaurus-based web book
- Markdown chapters with code examples and Mermaid diagrams
- Integrated RAG chatbot for Q&A
- Mobile-responsive design

Please create a project constitution using `/sp.constitution` that establishes governing principles for this textbook project. The constitution should cover:

## 1. Educational Philosophy

**Guiding Principles:**
- **Explanation-First**: Always explain "why" before "how"
- **Progressive Complexity**: Simple concepts → Intermediate applications → Advanced systems
- **Conceptual Mastery**: Understanding principles over memorizing commands
- **Real-World Relevance**: Every concept tied to industry applications
- **Accessibility**: No hardware required, pure theoretical learning with complete examples

**Learning Approach:**
- Theory (40%) / Practice Examples (60%) balance
- Multiple mental models for same concept
- Analogies from everyday life
- Visual-first explanations

## 2. Content Quality Standards

**Writing Style:**
- **Tone**: Professional but approachable, like a knowledgeable mentor
- **Voice**: Active voice, present tense, second person ("you will learn")
- **Clarity**: Beginner-friendly without "dumbing down"
- **Sentence Length**: Average 15-20 words, maximum 30 words
- **Technical Terms**: Define immediately on first use
- **Avoid**: Passive voice, jargon without explanation, unnecessarily complex language

**Chapter Structure Requirements:**
- All chapters must follow identical template structure
- Word count: 2,500-4,000 words per chapter (target: 3,200)
- Sections required: Overview, Introduction (400 words), Core Concepts (1,500-2,000 words), Implementation Guide (800-1,200 words), Common Pitfalls, Real-World Applications, Conceptual Exercises, Key Takeaways, Further Reading, Next Chapter Preview
- Each chapter must be self-contained but build on previous knowledge

**Quality Bars:**
- Every chapter must address all stated learning objectives
- No placeholder content or "TODO" markers in final version
- All cross-references must be valid
- Mobile-responsive formatting required

## 3. Code Example Standards

**Completeness:**
- All code must be complete and runnable (in theory)
- No snippets or partial examples
- Every example must include imports, main function, and if-name-main guard
- Include example usage and expected output

**Documentation:**
- Module-level docstrings explaining purpose
- Class docstrings with attributes and examples
- Function docstrings with Args, Returns, Raises (Google style)
- Inline comments for complex logic (explain "why", not "what")
- Type hints required for all parameters and returns

**Code Standards:**
- **Python**: Follow PEP 8, use type hints, include error handling
- **ROS 2**: Follow ROS 2 conventions, proper node lifecycle
- **XML/URDF**: Well-commented robot descriptions
- **YAML**: Clear structure with inline explanations
- **Quality**: Production-ready patterns, not toy examples

**Complexity Levels:**
- Basic: 50-100 lines, single file, simple logic, minimal dependencies
- Intermediate: 100-200 lines, multiple classes, good error handling
- Advanced: 200-400 lines, production features, full logging and monitoring

**Quantity per Chapter:**
- Minimum: 8 code examples
- Maximum: 12 code examples
- Total: 150-300 lines of code per chapter

## 4. Visual Content Standards

**Diagram Requirements:**
- Minimum 5 diagrams per chapter
- Maximum 7 diagrams per chapter
- All diagrams must use Mermaid syntax (built into Docusaurus)
- Every diagram must have: context paragraph (before), caption (after), explanation paragraph (after)

**Diagram Types:**
- Flowchart: For algorithms, decision trees, processes
- Sequence: For node interactions, message passing, time-based events
- Architecture: For system components, layers, modules
- State: For robot modes, transitions, lifecycle
- Class: For software structure, inheritance, relationships

**Design Standards:**
- One concept per diagram maximum
- Left-to-right or top-to-bottom flow
- Consistent color scheme:
  - Blue (#e3f2fd): Hardware/Physical components
  - Orange (#fff3e0): Middleware/Communication
  - Purple (#f3e5f5): Application/Software
  - Green (#e1f5e1): Success/Start states
  - Red (#ffe1e1): Error/End states
  - Yellow (#fff4e1): Warning/Decision points
- Clear labels on all elements
- Include legends when needed
- Mobile-friendly (not too wide)

## 5. Exercise Standards

**Exercise Types (3-5 per chapter):**
- **Analysis Exercise**: Test conceptual understanding (given scenario, explain what happens)
- **Design Exercise**: Apply knowledge (design a system component)
- **Debugging Exercise**: Problem-solving (identify and fix issues)
- **Comparison Exercise**: Critical thinking (compare approaches, discuss trade-offs)
- **Optional Advanced**: Challenge problem for deeper exploration

**Exercise Quality:**
- Must test concepts from the chapter
- Appropriate difficulty for target audience (beginners to intermediate)
- Complete solutions provided with detailed explanations
- No hardware/simulation required
- Real-world relevance

**Solution Format:**
- Step-by-step explanation
- Key concepts highlighted
- Common mistakes addressed
- Why the solution works

## 6. Technical Implementation Standards

**Docusaurus Integration:**
- Chapters saved as: `docs/part-X-name/chapter-XX-title-slug.md`
- Frontmatter required: `sidebar_position`, `title`
- Mermaid diagrams must render correctly
- Code syntax highlighting for: Python, YAML, XML, Bash
- All internal links must use relative paths

**File Organization:**
- Part folders: `part-1-foundations`, `part-2-ros2`, `part-3-simulation`, `part-4-isaac`, `part-5-vla`, `part-6-deployment`
- Naming convention: `chapter-XX-kebab-case-title.md`
- Images (if any): `static/img/part-X/`

**Performance:**
- Chapter load time under 2 seconds
- All images optimized
- Mermaid diagrams optimized for rendering speed

## 7. Consistency & Maintenance

**Terminology:**
- Use consistent terms throughout (e.g., always "ROS 2" not "ROS2" or "ros2")
- Maintain glossary of technical terms
- First use of term: define it
- Subsequent uses: use term confidently

**Cross-References:**
- Link to prerequisite chapters when building on previous knowledge
- Forward references to future chapters when introducing foundational concepts
- All links must be valid

**Version Control:**
- Each chapter development in separate feature branch
- Clear commit messages
- Pull request for review before merging

## 8. Subagent Coordination

**Subagent Roles:**
- **Orchestrator**: Manages workflow, coordinates subagents, quality control
- **Content-Writer**: Writes explanations, uses analogies, creates exercises
- **Code-Architect**: Creates production-quality code examples with documentation
- **Diagram-Designer**: Creates Mermaid diagrams with consistent styling

**Collaboration Standards:**
- Orchestrator always validates output from other subagents
- Content-writer requests diagrams from diagram-designer (doesn't create them)
- Content-writer requests code from code-architect (doesn't write it)
- All subagents follow this constitution
- Quality checkpoints after each phase

## 9. Quality Assurance

**Before Marking Chapter Complete:**
- [ ] All learning objectives addressed
- [ ] Word count: 2,500-4,000
- [ ] Code examples: 8-12, all complete and documented
- [ ] Diagrams: 5-7, all render correctly
- [ ] Exercises: 3-5 with complete solutions
- [ ] All sections present per template
- [ ] Frontmatter correct
- [ ] Mobile-responsive
- [ ] No spelling/grammar errors
- [ ] All links work
- [ ] Follows all constitution standards

**Testing:**
- Chapter loads in Docusaurus without errors
- All diagrams render
- All code blocks have syntax highlighting
- Navigation works correctly
- Mobile view is readable

## 10. Constraints & Boundaries

**What We DON'T Do:**
- No hands-on hardware labs (pure theoretical learning)
- No actual robot simulation required (examples explain how it works)
- No installation guides for software (conceptual understanding only)
- No platform-specific instructions (focus on concepts, not setup)

**What We DO:**
- Provide complete code examples (that would work if student had environment)
- Explain concepts thoroughly with diagrams
- Show real-world applications and industry examples
- Prepare students for future hands-on work

**Scope:**
- 22 chapters total across 6 parts
- Each chapter self-contained but progressive
- Focus: Physical AI, ROS 2, Gazebo, NVIDIA Isaac, Vision-Language-Action, Humanoid Robotics

---

Please create a comprehensive constitution based on these principles that will govern all development decisions for the Physical AI textbook project. The constitution should be saved to `.specify/memory/constitution.md` and serve as the foundational reference for all subsequent specification, planning, and implementation work.
```

## Response snapshot

The project constitution for the Physical AI & Humanoid Robotics Textbook has been successfully created and saved to `.specify/memory/constitution.md`. This constitution establishes governing principles for educational philosophy, content quality, code examples, visual content, exercises, technical implementation, consistency, subagent coordination, quality assurance, and project constraints. A Sync Impact Report was generated, indicating that all dependent templates will be dynamically informed by this new constitution.

## Outcome

- ✅ Impact: Created foundational constitution for the project.
- 🧪 Tests: N/A
- 📁 Files: 1 file created: .specify/memory/constitution.md
- 🔁 Next prompts: User to review constitution and proceed with spec/plan.
- 🧠 Reflection: Successfully established project governance and standards.
- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment: N/A
