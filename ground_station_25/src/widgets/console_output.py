import sys

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QTextEdit

from syntax_highlighters.console import ConsoleHighlighter


class EmittingStream(QtCore.QObject):
    """
    A custom stream that emits text written to it as a signal.

    Inherits
    --------
    `QObject`

    Attributes
    ----------
    textWritten : pyqtSignal
        Signal emitted when text is written to the stream.
    """

    textWritten = QtCore.pyqtSignal(str)

    def write(self, text) -> None:
        """
        Write text to the stream and emit a signal with the text.
        Parameters
        ----------
        text : str
            The text to write to the stream.
        """

        self.textWritten.emit(str(text))

    def flush(self) -> None:
        """
        Flush the stream. This method is required for compatibility with
        standard output streams.
        """

        pass


class ConsoleOutputWidget(QWidget):
    """
    A widget for displaying console output in a text edit with syntax highlighting.

    Inherits
    --------
    `QWidget`
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        self.original_stdout = sys.stdout
        self.original_stderr = sys.stderr

        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        self.console_output = QTextEdit()
        self.console_output.setReadOnly(True)
        self.console_output.setLineWrapMode(QTextEdit.NoWrap)
        self.main_layout.addWidget(self.console_output)

        self.highlighter = ConsoleHighlighter(self.console_output.document())

        self.stdout_stream = EmittingStream()
        self.stderr_stream = EmittingStream()

        sys.stdout = self.stdout_stream
        sys.stderr = self.stderr_stream

        self.stdout_stream.textWritten.connect(self.append_text)
        self.stderr_stream.textWritten.connect(self.append_text)

    def append_text(self, text: str) -> None:
        """Append text to the console output widget only."""

        cursor = self.console_output.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
        cursor.insertText(text)
        self.console_output.setTextCursor(cursor)
        self.console_output.ensureCursorVisible()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        """Restore original streams when widget is closed."""

        sys.stdout = self.original_stdout
        sys.stderr = self.original_stderr
        super().closeEvent(event)
