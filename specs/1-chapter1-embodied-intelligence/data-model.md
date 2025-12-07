# Data Model: Chapter 1 - The Dawn of Embodied Intelligence

**Feature**: `1-chapter1-embodied-intelligence`
**Date**: 2025-12-07

## Entity Overview

This chapter is a documentation artifact, not a software feature. The "data model" describes the structure and components of the chapter content.

---

## Primary Entity: Chapter

```yaml
Chapter:
  id: "chapter-01"
  title: "The Dawn of Embodied Intelligence"
  part: 1
  difficulty: "beginner"
  level: 1

  metadata:
    sidebar_position: 1
    word_count_target: 1800-2500
    reading_time: "20-25 minutes"
    prerequisites: null

  sections:
    - introduction
    - core_concept_1
    - core_concept_2
    - core_concept_3
    - implementation_perspective
    - common_pitfalls
    - real_world_applications
    - exercises
    - key_takeaways
    - further_reading
    - next_chapter_preview

  learning_objectives:
    - id: LO-001
      description: "Define Physical AI and explain how it differs from digital AI"
      tested_by: [exercise_1, core_concept_1]
    - id: LO-002
      description: "Identify scenarios where Physical AI is essential vs where digital AI is sufficient"
      tested_by: [exercise_1]
    - id: LO-003
      description: "Understand why humanoid robots are particularly suited for human-centered environments"
      tested_by: [core_concept_2]
    - id: LO-004
      description: "Recognize key components of a Physical AI system"
      tested_by: [exercise_2, core_concept_3]
    - id: LO-005
      description: "Feel excited and motivated about Physical AI"
      tested_by: [introduction, real_world_applications]
```

---

## Component Entities

### Code Example

```yaml
CodeExample:
  attributes:
    id: string           # e.g., "example-01-chatbot"
    title: string        # e.g., "Simple Chatbot (Digital AI)"
    language: "python"
    version: "3.8+"
    lines_target: 30-50
    purpose: string
    concept_demonstrated: string
    dependencies: []     # Empty for Chapter 1

  structure:
    - module_docstring: required
    - imports: optional
    - class_definition: optional
    - function_definitions: required
    - main_function: required
    - if_name_guard: required
    - expected_output_comment: required

  quality_requirements:
    - type_hints: required
    - docstrings: google_style
    - pep8_compliant: true
    - runnable_standalone: true
```

**Instances for Chapter 1:**

| ID | Title | Lines | Concept |
|----|-------|-------|---------|
| example-01-chatbot | Simple Chatbot | 30-40 | Digital AI simplicity |
| example-02-robot-state | Robot State Representation | 40-50 | Physical state tracking |
| example-03-sensor-data | Sensor Data Structure | 30-40 | Physical sensor complexity |

---

### Diagram

```yaml
Diagram:
  attributes:
    id: string           # e.g., "diagram-01-digital-vs-physical"
    title: string
    type: "graph"        # Only 'graph' for beginner chapters
    direction: "TB" | "LR" | "TD"
    purpose: string
    concept_illustrated: string

  structure:
    context_paragraph: string    # Before diagram
    mermaid_code: string         # The diagram itself
    caption: string              # Figure X: description
    explanation_paragraph: string # After diagram

  styling:
    colors:
      hardware: "#e3f2fd"
      middleware: "#fff3e0"
      software: "#f3e5f5"
      success: "#e1f5e1"
      error: "#ffe1e1"
      warning: "#fff4e1"
    max_width_nodes: 6
    label_max_words: 4
```

**Instances for Chapter 1:**

| ID | Title | Type | Direction |
|----|-------|------|-----------|
| diagram-01-digital-vs-physical | Digital AI vs Physical AI Architecture | graph | TB |
| diagram-02-humanoid-advantages | Humanoid Form Factor Advantages | graph | TD |
| diagram-03-system-architecture | Physical AI System Stack | graph | TB |
| diagram-04-control-loop | Control Loop Comparison (optional) | graph | LR |

---

### Exercise

```yaml
Exercise:
  attributes:
    id: string           # e.g., "exercise-01-scenario-analysis"
    title: string
    type: "analysis" | "design" | "debugging" | "comparison"
    difficulty: "beginner"

  structure:
    prompt: string
    context: string
    items: list          # For multi-part exercises

  solution:
    format: "collapsible"
    content: string
    explanation: string
    key_concepts: list
    common_mistakes: list
```

**Instances for Chapter 1:**

| ID | Title | Type | Items |
|----|-------|------|-------|
| exercise-01-scenario-analysis | Scenario Analysis | analysis | 5 scenarios |
| exercise-02-component-identification | Component Identification | analysis | 1 task breakdown |

---

### Section

```yaml
Section:
  attributes:
    id: string
    title: string
    word_count_target: integer
    required: boolean

  contains:
    - paragraphs: list[string]
    - code_examples: list[CodeExample]
    - diagrams: list[Diagram]
    - tables: list[Table]
    - lists: list[BulletList]
```

**Sections for Chapter 1:**

| Section | Word Target | Code | Diagram | Required |
|---------|-------------|------|---------|----------|
| Introduction | 300-350 | 0 | 0 | Yes |
| Core Concept 1 | 400-450 | 1 | 1 | Yes |
| Core Concept 2 | 350-400 | 1 | 1 | Yes |
| Core Concept 3 | 400-450 | 1 | 1 | Yes |
| Implementation Perspective | 200-250 | 0 | 0-1 | Yes |
| Common Pitfalls | 150-200 | 0 | 0 | Yes |
| Real-World Applications | 100-150 | 0 | 0 | Yes |
| Exercises | ~100 | 0 | 0 | Yes |
| Key Takeaways | ~50 | 0 | 0 | Yes |
| Further Reading | ~50 | 0 | 0 | Yes |
| Next Chapter Preview | ~50 | 0 | 0 | Yes |

---

## Relationships

```
Chapter 1:1 Frontmatter
Chapter 1:N Section
Section 0:N CodeExample
Section 0:N Diagram
Section 0:N Exercise
Exercise 1:1 Solution
LearningObjective N:M Section (tested_by relationship)
```

---

## Validation Rules

### Chapter-Level

1. `SUM(section.word_count)` MUST be in range [1800, 2500]
2. `COUNT(code_examples)` MUST equal 3
3. `COUNT(diagrams)` MUST be in range [3, 4]
4. `COUNT(exercises)` MUST equal 2
5. All `learning_objectives` MUST have at least one `tested_by` reference

### Code Example-Level

1. Each example MUST include `if __name__ == "__main__":`
2. Each example MUST have module docstring
3. Each function MUST have type hints
4. Each class/function MUST have Google-style docstring
5. Total lines MUST be in range specified for example

### Diagram-Level

1. Each diagram MUST have `context_paragraph` before
2. Each diagram MUST have `caption` after
3. Each diagram MUST have `explanation_paragraph` after
4. All colors MUST match constitution color scheme
5. `direction` MUST be one of [TB, TD, LR]

### Exercise-Level

1. Each exercise MUST have collapsible solution
2. Solution MUST include step-by-step explanation
3. Solution MUST highlight key concepts
4. Solution MUST address common mistakes

---

## File Structure

```
docs/part-1-foundations/
└── chapter-01-embodied-intelligence.md
    ├── frontmatter (yaml)
    ├── # Chapter 1: The Dawn of Embodied Intelligence
    ├── ## Overview (metadata box)
    ├── ## Introduction
    ├── ## Digital AI vs Physical AI
    │   ├── text
    │   ├── diagram-01
    │   └── example-01
    ├── ## Why Humanoid Robots?
    │   ├── text
    │   ├── diagram-02
    │   └── example-02
    ├── ## Physical AI System Components
    │   ├── text
    │   ├── diagram-03
    │   └── example-03
    ├── ## Implementation Perspective
    │   └── (optional diagram-04)
    ├── ## Common Pitfalls
    │   └── table
    ├── ## Real-World Applications
    ├── ## Conceptual Exercises
    │   ├── exercise-01 + solution
    │   └── exercise-02 + solution
    ├── ## Key Takeaways
    ├── ## Further Reading
    └── ## Next Chapter Preview
```
