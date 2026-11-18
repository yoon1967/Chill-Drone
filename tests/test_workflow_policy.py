"""Local regression checks for the retained Jira integration templates."""

import json
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class AutomationTemplateTests(unittest.TestCase):
    def test_callback_json_preserves_quotes_backslashes_and_newlines(self):
        title = 'infra: [S15P31B205-14] 협업 규칙 "인용"'
        description = 'Jira: https://ssafy.atlassian.net/browse/S15P31B205-14\n경로 C:\\test\n"검증"'
        path = ROOT / "automation/jira-mr/gitlab-update-body.json.template"
        rendered = path.read_text(encoding="utf-8").replace(
            "{{updatedMrTitle.jsonEncode}}", json.dumps(title, ensure_ascii=False)[1:-1]
        ).replace(
            "{{updatedMrDescription.jsonEncode}}", json.dumps(description, ensure_ascii=False)[1:-1]
        )
        self.assertEqual(
            json.loads(rendered),
            {"title": title, "description": description, "squash": True},
        )

    def test_default_mr_template_does_not_supply_an_example_issue_key(self):
        template = (ROOT / ".gitlab/merge_request_templates/Default.md").read_text(encoding="utf-8")
        # Jira Automation extracts keys from the entire body, including comments.
        self.assertEqual(re.findall(r"[A-Z][A-Z0-9_]*-[1-9][0-9]*", template), [])


if __name__ == "__main__":
    unittest.main()
