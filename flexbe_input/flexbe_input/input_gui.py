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
from PySide6.QtWidgets import QLabel, QLineEdit, QMainWindow, QPushButton, QVBoxLayout, QWidget


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

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        central_widget.setStyleSheet('QWidget { border: 1px solid blue; background-color: palette(window); }')

        layout = QVBoxLayout(central_widget)

        self.prompt = QLabel(self)
        self.prompt.setText(prompt)
        self.prompt.setStyleSheet('QLabel { border: none; background-color: palette(window); }')
        layout.addWidget(self.prompt)

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

        self.line = QLineEdit(self)
        self.line.setStyleSheet(edit_style)
        self.line.returnPressed.connect(self.set_input)  # Treat return as submit
        layout.addWidget(self.line)
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
        self.submit.clicked.connect(self.set_input)
        layout.addWidget(self.submit)

        self.cancel = QPushButton('Cancel', self)
        self.cancel.setStyleSheet(button_style)
        self.cancel.clicked.connect(self.set_cancel)
        layout.addWidget(self.cancel)

        self.adjustSize()

    def set_input(self):
        """Set input text from GUI."""
        self.input = self.line.text()

    def set_cancel(self):
        """Set input text from GUI."""
        self.input = ''

    @Slot(str)
    def show(self, prompt):
        """Show dialog if hidden."""
        print(f"showing input UI dialog with '{prompt}' ", flush=True)
        self.prompt.setText(prompt)
        self.prompt.adjustSize()
        self.line.setText('')
        self.input = None  # clear for next entry
        self.adjustSize()
        self.resize(self.sizeHint())  # Resize to fit the new content
        super().show()

    @Slot()
    def hide(self):
        """Hide dialog when not in use."""
        print('hiding input UI dialog', flush=True)
        super().hide()

    def is_none(self):
        """Return true while input is none."""
        return self.input is None

    def get_input(self):
        """Get the stored input."""
        return self.input
