"""LLM clients for the autoresearch harness (all three levels use the same LLM —
gains come from loop ARCHITECTURE, not a stronger meta-model; CLAUDE.md).

Two implementations behind one duck-typed interface:
  * AnthropicLLM — the real thing (lazy SDK import; adaptive thinking;
    structured outputs for Level-1 proposals). Default model claude-opus-4-8.
  * ScriptedLLM — deterministic canned responses for tests/CI/G5, so the gate
    and the test suite never need an API key or network.

Interface:
    complete_json(system, prompt, schema) -> dict   (schema-constrained)
    complete_text(system, prompt) -> str            (free text, Level-2 rounds)
"""
import json


class AnthropicLLM:
    def __init__(self, model: str = "claude-opus-4-8", max_tokens: int = 16000):
        import anthropic                       # lazy: harness runs without SDK
        self.client = anthropic.Anthropic()
        self.model = model
        self.max_tokens = max_tokens

    def complete_json(self, system: str, prompt: str, schema: dict) -> dict:
        response = self.client.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            thinking={"type": "adaptive"},
            system=system,
            output_config={"format": {"type": "json_schema", "schema": schema}},
            messages=[{"role": "user", "content": prompt}],
        )
        text = next(b.text for b in response.content if b.type == "text")
        return json.loads(text)

    def complete_text(self, system: str, prompt: str) -> str:
        with self.client.messages.stream(
            model=self.model,
            max_tokens=64000,                  # Level-2 Generate can be long
            thinking={"type": "adaptive"},
            system=system,
            messages=[{"role": "user", "content": prompt}],
        ) as stream:
            response = stream.get_final_message()
        return next(b.text for b in response.content if b.type == "text")


class ScriptedLLM:
    """Deterministic double: pops canned responses in order. Runs dry (raises)
    rather than inventing output — a test that outruns its script is a bug."""

    def __init__(self, json_responses=None, text_responses=None):
        self.json_responses = list(json_responses or [])
        self.text_responses = list(text_responses or [])
        self.calls = []                        # (kind, system, prompt) audit

    def complete_json(self, system, prompt, schema):
        self.calls.append(("json", system, prompt))
        if not self.json_responses:
            raise RuntimeError("ScriptedLLM ran out of json responses")
        return self.json_responses.pop(0)

    def complete_text(self, system, prompt):
        self.calls.append(("text", system, prompt))
        if not self.text_responses:
            raise RuntimeError("ScriptedLLM ran out of text responses")
        return self.text_responses.pop(0)
