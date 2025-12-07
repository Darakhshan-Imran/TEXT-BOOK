# Skill: Generate Chapter Structure

## Purpose
Creates consistent chapter structure following the Physical AI textbook template. This skill ensures every chapter has the same professional format and all required sections.

## Capabilities
- Generates complete chapter markdown with frontmatter
- Creates all standard sections (Overview, Introduction, Core Concepts, etc.)
- Includes placeholders for content
- Maintains consistent formatting

## Input Parameters

When using this skill, provide:

1. **chapter_number**: Integer (1-22)
2. **chapter_title**: String (e.g., "The Dawn of Embodied Intelligence")
3. **part_folder**: String (e.g., "part-1-foundations")
4. **learning_objectives**: Array of 3-5 learning goals
5. **prerequisites**: Array of prerequisite chapters (can be empty for Chapter 1)
6. **estimated_time**: String (e.g., "25-30 minutes")

## Output Format

The skill generates a markdown file with this structure:

```markdown
---
sidebar_position: {chapter_number}
title: "Chapter {chapter_number}: {chapter_title}"
---

# Chapter {chapter_number}: {chapter_title}

## 📖 Overview

**Learning Objectives:**

{List each learning objective as bullet point}

**Prerequisites:**
{List prerequisite chapters with links, or "None" if first chapter}

**Estimated Reading Time:** {estimated_time}

---

## 🎯 Introduction

[Introduction content - 400 words explaining why this chapter matters]

---

## 📚 Core Concepts

[Main teaching content with subsections - 1,500-2,000 words]

### Concept 1: [Name]

**What it is:** [Definition]

**Why it matters:** [Relevance]

**How it works:** [Detailed explanation]

### Concept 2: [Name]

[Same structure]

### Concept 3: [Name]

[Same structure]

---

## 💻 Implementation Guide

[Code examples and practical demonstrations - 800-1,200 words]

### Example 1: [Basic Implementation]

[Code block with comments]

**Explanation:** [What the code does]

### Example 2: [Intermediate Implementation]

[Code block with comments]

**Explanation:** [Integration details]

---

## 🔍 Understanding the Architecture

[System diagrams and architectural explanations]

---

## ⚠️ Common Pitfalls & Solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| [Issue 1] | [Why it happens] | [How to fix] |
| [Issue 2] | [Why it happens] | [How to fix] |
| [Issue 3] | [Why it happens] | [How to fix] |

---

## 🌍 Real-World Applications

**Industry Example 1:** [Company/Project and how they use this]

**Industry Example 2:** [Research or practical deployment]

**Industry Example 3:** [Current trends and applications]

---

## 🧠 Conceptual Exercises

### Exercise 1: Analysis
[Question that requires understanding concepts]

### Exercise 2: Design
[Question that requires applying knowledge]

### Exercise 3: Debugging
[Question about identifying and fixing issues]

**Solutions:**
[Detailed solutions for each exercise]

---

## 📝 Key Takeaways

- ✓ [Key point 1]
- ✓ [Key point 2]
- ✓ [Key point 3]
- ✓ [Key point 4]
- ✓ [Key point 5]

---

## 📚 Further Reading

- **[Resource 1]**: [Description of what you'll learn]
- **[Resource 2]**: [Description]
- **[Resource 3]**: [Description]
- **[Resource 4]**: [Description]

---

## ➡️ Next Chapter Preview

[2-3 sentences about the next chapter and how it builds on this one]
```

## Usage Example

To use this skill, provide parameters like this:

```yaml
chapter_number: 1
chapter_title: "The Dawn of Embodied Intelligence"
part_folder: "part-1-foundations"
learning_objectives:
  - "Understand the difference between digital AI and Physical AI"
  - "Explain why humanoid robots are suited for human environments"
  - "Identify key components of Physical AI systems"
  - "Recognize the importance of embodied intelligence"
prerequisites: []
estimated_time: "25-30 minutes"
```

## Quality Standards

Every chapter generated with this skill must:

1. **Be Complete**: All sections must have content, no empty placeholders
2. **Be Consistent**: Follow the exact template structure
3. **Be Educational**: Explain concepts clearly with examples
4. **Be Visual**: Include diagrams and code examples
5. **Be Progressive**: Build on previous knowledge appropriately

## File Naming Convention

Output files should follow this pattern:
`docs/{part_folder}/chapter-{number:02d}-{title-slug}.md`

Example: `docs/part-1-foundations/chapter-01-embodied-intelligence.md`

## Integration with Other Skills

This skill works together with:
- **code-example**: To fill in the Implementation Guide section
- **diagram-generator**: To create visuals for architecture sections
- **content-writer**: To write the actual content for each section