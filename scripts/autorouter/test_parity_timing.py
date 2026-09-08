"""Keep router time distinct from JVM/process startup and host validation."""
import unittest

from run_parity_case import reference_routing_ms


class ReferenceTimingTests(unittest.TestCase):
    def test_sums_completed_stages_not_passes_or_job_wall_time(self):
        log = """Fanout stage completed: started with 10 pins, completed in 0.25 seconds
Auto-routing pass #1 completed in 0.40 seconds
Auto-routing stage completed: started with 8 nets, completed in 1.50 seconds
Job finished (elapsed: 2.50 seconds)
"""
        self.assertEqual(reference_routing_ms(log), 1750)

    def test_fanout_is_optional(self):
        self.assertEqual(reference_routing_ms(
            "Auto-routing stage completed: completed in 0.04 seconds"), 40)

    def test_missing_or_ambiguous_stages_are_not_zero_time(self):
        self.assertIsNone(reference_routing_ms("Job failed"))
        self.assertIsNone(reference_routing_ms(
            "Auto-routing stage completed: completed in 1.00 seconds\n" * 2))


if __name__ == "__main__":
    unittest.main()
