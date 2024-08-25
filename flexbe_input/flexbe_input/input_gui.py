# Copyright 2024 Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""FlexBE InputGUI."""
from PySide6.QtCore import QSize, Slot
from PySide6.QtWidgets import QComboBox, QLabel, QLineEdit, QMainWindow, QPushButton, QVBoxLayout, QWidget


class InputGUI(QMainWindow):
    """
    The GUI for input_action_server.

    Instances of this class should be created in input_action_server.py.
    """

    def __init__(self, prompt):
        """Initialize the InputGUI instance."""
        QMainWindow.__init__(self)

        self.input = None

        self.setMinimumSize(QSize(320, 180))
        self.setWindowTitle('FlexBE Input State')
        self.setStyleSheet("""
            QMainWindow {
                border: 2px solid #8f8f91;
                border-radius: 10px;
            }
            """)

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        self.central_widget.setStyleSheet('QWidget { border: 1px solid blue; background-color: palette(window); }')
        self.combo_box = None
        self.input_line = None

    def clear_layout(self):
        """Clear prior layout of widget."""
        while self.main_layout.count():
            item = self.main_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)  # Detach the widget from the layout
                widget.deleteLater()

    def set_layout(self, prompt, items=None):
        """Update the widget layout."""
        self.clear_layout()

        self.prompt = QLabel(self)
        self.prompt.setStyleSheet('QLabel { border: none; background-color: palette(window); }')
        self.prompt.setText(prompt)
        self.prompt.adjustSize()
        self.main_layout.addWidget(self.prompt)

        if items is None:
            # One line entry field
            edit_style = """
            QLineEdit {
                border: 2px solid #8f8f91;
                background-color: #f0f0f0;
                padding: 2px;
                color: black;
            }

            QLineEdit:focus {
                border: 2px solid #0078d7;  /* Change this color to your desired highlight color */
            }
            """

            self.input_line = QLineEdit(self)
            self.input_line.setStyleSheet(edit_style)
            self.input_line.returnPressed.connect(self.set_input)  # Treat return as submit
            self.input_line.setText('')
            self.combo_box = None
            self.main_layout.addWidget(self.input_line)
        else:
            combo_style = """
                QComboBox {
                    border: 2px solid #8f8f91;
                    background-color: #f0f0f0;
                    padding: 2px;
                    color: black;
                }

                QComboBox:focus {
                    border: 2px solid #0078d7;  /* Highlight color */
                }

                QComboBox::drop-down {
                    border-left: 1px solid #8f8f91;
                }

                QComboBox::down-arrow {
                    image: url(down_arrow.png);  /* Optionally customize the down arrow */
                    width: 10px;
                    height: 10px;
                }

                QComboBox QAbstractItemView {
                    background-color: #f0f0f0;
                    selection-background-color: #0078d7;
                    border: 1px solid #8f8f91;
                }
                """

            self.combo_box = QComboBox(self)
            self.combo_box.setStyleSheet(combo_style)
            string_items = []
            for item in items:
                if not isinstance(item, str):
                    string_items.append(f'{item}')  # Convert to string if not already
                else:
                    string_items.append(item)
            self.combo_box.addItems(string_items)
            self.combo_box.currentIndexChanged.connect(self.change_selection)
            self.line = None
            self.main_layout.addWidget(self.combo_box)

        button_style = """
        QPushButton {
            border: 2px solid #8f8f91;
            border-radius: 6px;
            background-color: #f0f0f0;
            padding: 2px;
            color: black;
        }

        QPushButton:pressed {
            background-color: #dcdcdc;
            border-style: inset;
        }
        QPushButton:focus {
            border: 2px solid #0078d7;  /* Change this color to your desired highlight color */
        }
        """
        self.submit = QPushButton('Submit', self)
        self.submit.setStyleSheet(button_style)
        if items is None:
            self.submit.clicked.connect(self.set_input)
        else:
            self.submit.clicked.connect(self.set_selection)

        self.main_layout.addWidget(self.submit)

        self.cancel = QPushButton('Cancel', self)
        self.cancel.setStyleSheet(button_style)
        self.cancel.clicked.connect(self.set_cancel)
        self.main_layout.addWidget(self.cancel)

        self.adjustSize()

    def change_selection(self):
        """Print selection."""
        if self.combo_box is None:
            print('Unknown combo box - why is this called!', flush=True)
        else:
            print(f" Currently selected '{self.combo_box.currentText()}' ")

    def set_selection(self):
        """Set input text from selection box."""
        if self.combo_box is None:
            print('Unknown combo box - why is this called!', flush=True)
            self.input = 'unknown'
        else:
            print(f" Selected '{self.combo_box.currentText()}' ")
            self.input = self.combo_box.currentText()

    def set_input(self):
        """Set input text from GUI."""
        if self.input_line is None:
            print('Unknown combo box - why is this called!', flush=True)
            self.input = 'unknown'
        else:
            self.input = self.input_line.text()

    def set_cancel(self):
        """Set input text from GUI."""
        self.input = ''

    @Slot(str)
    def show(self, prompt, items=None):
        """Show dialog if hidden."""
        print(f"showing input UI dialog with '{prompt}' ", flush=True)
        self.input = None  # clear for next entry
        self.set_layout(prompt, items)
        self.resize(self.sizeHint())  # Resize to fit the new content
        super().show()

    @Slot()
    def hide(self):
        """Hide dialog when not in use."""
        print('hiding input UI dialog', flush=True)
        self.clear_layout()
        super().hide()

    def is_none(self):
        """Return true while input is none."""
        return self.input is None

    def get_input(self):
        """Get the stored input."""
        return self.input
