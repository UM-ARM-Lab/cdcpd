import unittest

from arc_utilities.algorithms import is_list_unique, nested_dict_update


class TestAlgorithms(unittest.TestCase):

    def test_is_lis_unique(self):
        self.assertTrue(is_list_unique([1, 2, 3]))
        self.assertTrue(is_list_unique(['a', 2, 3]))
        self.assertTrue(is_list_unique(['a', 2, None]))

        self.assertFalse(is_list_unique(['a', 2, None, 2]))
        self.assertFalse(is_list_unique(['a', 'a', None, 2]))
        self.assertFalse(is_list_unique([8, 'a', None, None]))

    def test_nested_dict_update(self):
        base_dict = {
            'a': 1,
            'b': [2, 3],
            'd': [2, 3],
            'c': {
                'x': "x",
                'y': "y",
            },
            'e': None,
        }

        update_dict = {
            'a': 100,
            'd': [4, 5, 6],
            'c': {
                'x': "not_x",
            },
            'f': 1,
        }
        updated_dict = nested_dict_update(base_dict, update_dict)

        self.assertEqual(updated_dict['a'], 100)
        self.assertEqual(updated_dict['b'], [2, 3])
        self.assertEqual(updated_dict['c']['x'], "not_x")
        self.assertEqual(updated_dict['c']['y'], "y")
        self.assertEqual(updated_dict['d'], [4, 5, 6])
        self.assertEqual(updated_dict['e'], None)
        self.assertEqual(updated_dict['f'], 1)


if __name__ == '__main__':
    unittest.main()
