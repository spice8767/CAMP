# Antigravity Global Agent Rules

## Always-On Response Footer: Telemetry & Context Window Panel

At the end of every response, you MUST append a formatted Context Window & Telemetry footer panel matching this exact structure:

```
┌─ Context Window ────────────────────────────────────────────────────────┐
│ [{PROGRESS_BAR}] {PERCENT}% ({CURRENT_TOKENS:,} / {MAX_TOKENS:,} tokens)      │
├─────────────────────────────────────────────────────────────────────────┤
│ Turn: {TURN_IN:,} in · {TURN_OUT:,} out | Cache Hit: {CACHE_HIT}% | Est. Cost: ${SESSION_COST:.3f} (Session) │
└─────────────────────────────────────────────────────────────────────────┘
```

### Guidelines for Telemetry Values:
1. **Progress Bar**: 32 slots total (`█` for filled, `░` for remaining).
2. **Context Window**:
   - `MAX_TOKENS`: Hard limit of the active model (e.g. 1,048,576 for Gemini Flash/Pro, 200,000 for Claude, 128,000 for GPT-4o).
   - `CURRENT_TOKENS`: Estimated cumulative tokens loaded in context for the active session.
   - `PERCENT`: Calculated as `(CURRENT_TOKENS / MAX_TOKENS) * 100`.
3. **Turn & Session Stats**:
   - `TURN_IN`: Fresh + cached prompt tokens for this turn.
   - `TURN_OUT`: Tokens generated in this response.
   - `CACHE_HIT`: Percentage of prompt tokens reused from provider cache.
   - `SESSION_COST`: Estimated accumulated session cost in USD.
