import unittest

from arc_utilities.conversions import parse_file_size


class TestConversions(unittest.TestCase):

    def test_parse_file_size(self):
        self.assertEqual(parse_file_size('1'), 1)
        self.assertEqual(parse_file_size('12.2'), 12)
        self.assertEqual(parse_file_size('1k'), 1_000)
        self.assertEqual(parse_file_size('1.2mB'), 1_200_000)
        self.assertEqual(parse_file_size('1Gb'), 1_000_000_000)
        self.assertEqual(parse_file_size('3K'), 3_000)
        self.assertEqual(parse_file_size('3M'), 3_000_000)

        with self.assertRaises(ValueError):
            parse_file_size("hello")


if __name__ == '__main__':
    unittest.main()