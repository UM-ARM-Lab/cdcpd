import unittest

from arc_utilities.conditional_try import conditional_try


def _raises(s):
    raise ValueError("error: " + s)


def _not_raises(s):
    return "no error: " + s


class TestConditionalTry(unittest.TestCase):

    def test_raises(self):
        with self.assertRaises(ValueError):
            conditional_try(should_catch=False, function=_raises, s="test")
            conditional_try(should_catch=False, function=_raises, s="test", value_on_exception=7)

    def test_no_raise(self):
        self.assertEqual(conditional_try(False, _not_raises, s="test"), "no error: test")
        self.assertEqual(conditional_try(False, _not_raises, s="test", value_on_exception="hello"), "no error: test")

    def test_no_raises_catch(self):
        self.assertEqual(conditional_try(True, _raises, s="test", value_on_exception=True), True)
        self.assertEqual(conditional_try(True, _not_raises, s="test", value_on_exception=False), "no error: test")


if __name__ == '__main__':
    unittest.main()
