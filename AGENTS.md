# AGENTS.md — customOptiBoot

## Purpose
This folder (`customOptiBoot/`) contains the **source files of our custom OptiBoot bootloader**.  
They are included here so that Codex and other project tools can use them as an **information source** when reasoning about problems in the project.

## Usage Policy
- **Do not modify these files.**  
  They are frozen and must remain identical to the verified bootloader sources.

- **Intended use:**  
  - When Codex or any other system needs to understand how the bootloader works, it should **read the files in this folder**.  
  - These files provide definitions, constants, and logic that may help explain behaviors in the wider project.  
  - They are not to be compiled or linked directly by `.ino` sketches — they exist for **reference only**.

## Instruction to Codex
- Treat this folder as a **read-only knowledge base**.  
- You may consult these files to extract details (e.g., UART, baud rate, LED behavior, or memory layout).  
- You must **never generate, propose, or apply edits** to any file inside this folder.  
- If you need to reference bootloader details while solving a problem elsewhere, **use this folder as your source**.

## Why Locked?
- These files define a critical piece of hardware infrastructure.  
- Changes could cause bricked devices or non-reproducible builds.  
- Therefore, they are locked and may only be updated through a controlled, manual regeneration process.

---

## Summary
- **Folder role:** Knowledge source for project analysis  
- **Status:** Frozen, non-editable  
- **Codex behavior:** Use this folder to inform solutions, never to make changes  
