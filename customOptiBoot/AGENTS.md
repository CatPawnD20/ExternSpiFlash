# AGENTS.md — customOptiBoot

## Purpose
This folder contains the source set of our **custom OptiBoot**.  
These files exist in the repository **only as references** for scripts and `.ino` sketches.  

## Strict Rules
1. **These files must never be modified:**
   - `boot.h`  
   - `exup.c`  
   - `exup.h`  
   - `optiboot_flash.c`  
   - `Makefile`

2. They are here **to be read and referenced only**.  
   - Example: scripts may check configuration flags or build commands from `Makefile`.  
   - Sketches may analyze header definitions for consistency.  

3. **Instruction to Codex:**  
   - You may use these files as a **source of truth** when solving project problems.  
   - You must **never propose edits or direct modifications** to these files.

## Why Locked?
- These files define the project’s bootloader.  
- Any unreviewed change could brick the device or break reproducibility.  
- For that reason, they are treated as **frozen sources**.

---

## Safeguard
If Codex (or any automated system) attempts to **edit files in this folder**,  
it must be treated as an **error** and immediately stopped.  
Only manual updates through a controlled bootloader regeneration process are allowed.

---

## Summary
- **Role:** Reference / source of truth  
- **Status:** Frozen, non-editable  
- **Usage:** Read-only, for analysis and context only  
