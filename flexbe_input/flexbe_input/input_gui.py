# Copyright 2023 Christopher Newport University
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

from PyQt5.QtWidgets import QMainWindow, QLabel, QLineEdit, QPushButton
from PyQt5.QtCore import QSize


class InputGUI(QMainWindow):
    """
    The GUI for input_action_server.

    Instances of this class should be created in input_action_server.py.
    """
    def __init__(self, prompt):
        QMainWindow.__init__(self)

        self.input = None

        self.setMinimumSize(QSize(320, 140))
        self.setWindowTitle("Input State")

        self.prompt = QLabel(self)
        self.prompt.move(60, 20)
        self.prompt.setText(prompt)
        self.prompt.adjustSize()

        self.line = QLineEdit(self)
        self.line.move(60, 60)
        self.line.resize(200, 32)
        self.line.returnPressed.connect(self.set_input)  # Treat return as submit

        self.button = QPushButton('Submit', self)
        self.button.clicked.connect(self.set_input)
        self.button.resize(200, 32)
        self.button.move(60, 100)
        self.adjustSize()

    def set_input(self):
        self.input = self.line.text()

    def is_none(self):
        if self.input is None:
            return True
        return False

    def get_input(self):
        return self.input
