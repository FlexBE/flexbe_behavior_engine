#!/usr/bin/env python3

"""Unit tests for the FlexBE input dialog."""

import unittest

from flexbe_input.input_gui import InputGUI

from PySide6.QtWidgets import QApplication


class TestInputGUI(unittest.TestCase):
    """Validate headless dialog behavior."""

    @classmethod
    def setUpClass(cls):
        """Create one QApplication for the headless widget tests."""
        cls._app = QApplication.instance() or QApplication([])

    def test_line_input_layout_tracks_text_and_cancel(self):
        """Line-input mode should store typed text and let cancel clear it to an empty string."""
        gui = InputGUI('default')

        try:
            gui.set_layout('Enter value')
            self.assertEqual(gui.prompt.text(), 'Enter value')
            self.assertIsNotNone(gui.input_line)
            self.assertIsNone(gui.combo_box)
            self.assertTrue(gui.is_none())

            gui.input_line.setText('typed value')
            gui.set_input()
            self.assertEqual(gui.get_input(), 'typed value')
            self.assertFalse(gui.is_none())

            gui.set_cancel()
            self.assertEqual(gui.get_input(), '')
        finally:
            gui.hide()

    def test_selection_layout_stringifies_items_and_stores_choice(self):
        """Selection mode should stringify non-string items and capture the current choice."""
        gui = InputGUI('default')

        try:
            gui.set_layout('Choose value', items=['alpha', 2])
            self.assertEqual(gui.prompt.text(), 'Choose value')
            self.assertIsNotNone(gui.combo_box)
            self.assertEqual(gui.combo_box.count(), 2)
            self.assertEqual(gui.combo_box.itemText(0), 'alpha')
            self.assertEqual(gui.combo_box.itemText(1), '2')

            gui.combo_box.setCurrentIndex(1)
            gui.set_selection()
            self.assertEqual(gui.get_input(), '2')
        finally:
            gui.hide()

    def test_show_and_hide_reset_input_and_clear_widgets(self):
        """Showing should reset stale input and hiding should clear the active layout."""
        gui = InputGUI('default')

        try:
            gui.input = 'stale'
            gui.show('Prompt', items=['one'])
            self.assertTrue(gui.isVisible())
            self.assertIsNone(gui.get_input())
            self.assertEqual(gui.prompt.text(), 'Prompt')
            self.assertGreater(gui.main_layout.count(), 0)

            gui.hide()
            self.assertFalse(gui.isVisible())
            self.assertEqual(gui.main_layout.count(), 0)
        finally:
            gui.hide()


if __name__ == '__main__':
    unittest.main()
