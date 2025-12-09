# Content Structure Contract

**Feature Branch**: `001-physical-ai-book`
**Created**: 2025-12-09

## Overview

This document defines the content structure contracts for the Physical AI & Humanoid Robotics book. These contracts specify the required structure, metadata, and content elements for each type of document.

## Chapter Frontmatter Contract

Every chapter markdown file MUST include frontmatter adhering to this schema:

```yaml
---
sidebar_position: <integer>      # Required: Order within module (1-based)
title: "<string>"                # Required: Chapter title (max 60 chars)
description: "<string>"          # Required: SEO description (max 160 chars)
keywords: ["<string>"]           # Optional: Search keywords
---
```

**Validation Rules**:
- `sidebar_position`: Must be unique within the module directory
- `title`: Must not exceed 60 characters
- `description`: Must not exceed 160 characters
- All fields are case-sensitive

## Module Category Contract

Each module directory MUST contain a `_category_.json` file:

```json
{
  "label": "<Module Title>",
  "position": <integer>,
  "link": {
    "type": "generated-index",
    "description": "<Brief module description>"
  }
}
```

**Position Assignments**:
| Module | Position |
|--------|----------|
| Introduction | 1 |
| Module 1 (ROS 2) | 2 |
| Module 2 (Simulation) | 3 |
| Module 3 (Isaac) | 4 |
| Module 4 (VLA) | 5 |
| Capstone | 6 |
| Resources | 7 |

## Chapter Content Contract

Each chapter MUST include these sections in order:

### 1. Title (H1)
```markdown
# Chapter Title
```

### 2. Learning Outcomes Section
```markdown
## Learning Outcomes

After completing this chapter, you will be able to:
- [Measurable outcome 1 - use action verbs: explain, implement, configure, etc.]
- [Measurable outcome 2]
- [Measurable outcome 3]
```

**Requirements**:
- Minimum 3 outcomes per chapter
- Each outcome must start with an action verb
- Outcomes must be measurable/verifiable

### 3. Prerequisites Section (if applicable)
```markdown
## Prerequisites

Before starting this chapter, ensure you have completed:
- [Link to prerequisite chapter](/module-X-name/chapter-name)
- [Another prerequisite if needed]
```

### 4. Introduction Section
```markdown
## Introduction

[2-3 paragraphs introducing the topic and its relevance to Physical AI]
```

### 5. Main Content Sections
```markdown
## [Topic Section]

[Content with explanations, code examples, and diagrams]

### [Subsection if needed]

[Detailed content]
```

**Requirements**:
- Use H2 for major sections
- Use H3 for subsections
- Include at least 1 code example per chapter
- Include at least 1 diagram per module

### 6. Code Example Contract
````markdown
```<language> title="<filename>"
[code with inline comments]
```
````

**Supported Languages**:
- `python` - Python code
- `cpp` - C++ code
- `bash` - Shell commands
- `xml` - XML/URDF files
- `yaml` - YAML configuration
- `json` - JSON configuration

**Requirements**:
- All code blocks MUST specify language
- Code examples SHOULD include title
- Complex code MUST include inline comments

### 7. Diagram Contract
````markdown
```mermaid
[Mermaid diagram code]
```
*Figure N: [Caption describing the diagram]*
````

**Supported Diagram Types**:
- `flowchart` - Process flows, data flows
- `sequenceDiagram` - Interaction sequences
- `classDiagram` - Entity relationships
- `stateDiagram` - State machines

### 8. Practical Exercise Section
```markdown
## Practical Exercise

### Exercise: [Title]

[Exercise description]

**Steps**:
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Expected Outcome**: [What the learner should achieve]
```

### 9. Self-Assessment Section
```markdown
## Self-Assessment

Test your understanding with these questions:

1. [Question 1]?
2. [Question 2]?
3. [Question 3]?

<details>
<summary>View Answers</summary>

1. [Answer 1]
2. [Answer 2]
3. [Answer 3]

</details>
```

**Requirements**:
- Minimum 3 questions per chapter
- Include answers in collapsible section

### 10. Summary Section
```markdown
## Summary

In this chapter, you learned:
- [Key takeaway 1]
- [Key takeaway 2]
- [Key takeaway 3]

## Next Steps

Continue to [Next Chapter](/module-X-name/next-chapter) to learn about [topic].
```

### 11. References Section
```markdown
## References

- [Official ROS 2 Documentation](https://docs.ros.org/)
- [NVIDIA Isaac Documentation](https://developer.nvidia.com/isaac)
```

## Cross-Reference Contract

Internal links MUST use this format:
```markdown
[Link Text](/module-folder-name/chapter-filename)
```

**Examples**:
- `[ROS 2 Architecture](/module-1-ros2/01-architecture)`
- `[Isaac Sim Introduction](/module-3-isaac/01-isaac-sim-intro)`

## Admonition Contract

Use Docusaurus admonitions for special callouts:

```markdown
:::tip
Helpful tip or best practice
:::

:::warning
Important warning about potential issues
:::

:::note
Additional information or clarification
:::

:::info
Background information or context
:::
```

## Content Quality Gates

Before merging any content, verify:

1. **Frontmatter**: Valid YAML with all required fields
2. **Sections**: All mandatory sections present
3. **Code Blocks**: All have language specified
4. **Links**: All internal links resolve
5. **Length**: Chapter meets word count target
6. **Outcomes**: 3+ measurable learning outcomes
7. **Assessment**: 3+ self-assessment questions
8. **Build**: `npm run build` passes without errors
