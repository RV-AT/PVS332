##################################################################################
#这个是prv332ide的图形界面
#支持篇riscv i a指令的代码高亮
#直接调用了轻量级rv汇编器高速内核
#具有较高的可移植性和多平台兼容性
#其中可以生成bin coe 和hex
#方便了仿真和烧录
#同时可以与烧录器进行远程连接接收烧录器传来数据
#（连接分为两种模式1是将电脑做服务器进行 连接）
#二是将烧录器做服务器电脑连接
#两者各有优劣在此实现了电脑做主机的连接
#同时利用烧录器固件的webserver网页可以获得更加多的对soc的操作以及调试
#另外利用webserver也可远程更新烧录器固件
#author = Adancurusul
#2019.11.13










from ass2hex import change_into_bin,change_into_hex,do_scan,split_str
import os
import sys
from PyQt5.QtCore import (QEvent, QFile, QFileInfo, QIODevice, QRegExp,
                          QTextStream,Qt)
from PyQt5.QtWidgets import (QAction, QApplication,  QFileDialog,
                             QMainWindow, QMessageBox, QTextEdit,QToolBar)
from PyQt5.QtGui import QFont, QIcon,QColor,QKeySequence,QSyntaxHighlighter,QTextCharFormat,QTextCursor
from threading import Thread
#import qrc_resources
#from PyQt5.QtNetwork import (QTcpSocket,)









class AssemblyHighlighter(QSyntaxHighlighter):

    Rules = []
    Formats = {}

    def __init__(self, parent=None):
        super(AssemblyHighlighter, self).__init__(parent)

        self.initializeFormats()
        KEYWORDS = ['.data','.text']
        '''
        KEYWORDS = ["and", "as", "assert", "break", "class",
                "continue", "def", "del", "elif", "else", "except",
                "exec", "finally", "for", "from", "global", "if",
                "import", "in", "is", "lambda", "not", "or", "pass",
                "print", "raise", "return", "try", "while", "with",
                "yield"]
        '''
        BUILTINS =  ['addi', 'lui', 'auipc', 'jal', 'jalr', 'beq', 'bne', 'blt', 'bge', 'bltu', 'bgeu', 'lb', 'lh', 'lw', 'lbu', 'lhu', 'sb', 'sh', 'sw', 'slti', 'sltiu', 'xori', 'ori', 'andi', 'slli', 'srli', 'srai', 'add', 'sub', 'sll', 'slt', 'sltu', 'xor', 'srl', 'sra', 'or', 'and', 'fence.i', 'ecall', 'ebreak', 'cssrrw', 'csrrs', 'csrrc', 'csrrwi', 'csrrsi', 'csrrci', 'ADDI', 'LUI', 'AUIPC', 'JAL', 'JALR', 'BEQ', 'BNE', 'BLT', 'BGE', 'BLTU', 'BGEU', 'LB', 'LH', 'LW', 'LBU', 'LHU', 'SB', 'SH', 'SW', 'SLTI', 'SLTIU', 'XORI', 'ORI', 'ANDI', 'SLLI', 'SRLI', 'SRAI', 'ADD', 'SUB', 'SLL', 'SLT', 'SLTU', 'XOR', 'SRL', 'SRA', 'OR', 'AND', 'FENCE.I', 'ECALL', 'EBREAK', 'CSSRRW', 'CSRRS', 'CSRRC', 'CSRRWI', 'CSRRSI', 'CSRRCI']



        CONSTANTS = ["False", "True", "None", "NotImplemented",
                     "Ellipsis"]

        AssemblyHighlighter.Rules.append((QRegExp(
                "|".join([r"\b%s\b" % keyword for keyword in KEYWORDS])),
                "keyword"))
        AssemblyHighlighter.Rules.append((QRegExp(
                "|".join([r"\b%s\b" % builtin for builtin in BUILTINS])),
                "builtin"))
        AssemblyHighlighter.Rules.append((QRegExp(
                "|".join([r"\b%s\b" % constant
                for constant in CONSTANTS])), "constant"))
        AssemblyHighlighter.Rules.append((QRegExp(
                r"\b[+-]?[0-9]+[lL]?\b"
                r"|\b[+-]?0[xX][0-9A-Fa-f]+[lL]?\b"
                r"|\b[+-]?[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?\b"),
                "number"))
        AssemblyHighlighter.Rules.append((QRegExp(
                r"\bPyQt4\b|\bQt?[A-Z][a-z]\w+\b"), "pyqt"))
        AssemblyHighlighter.Rules.append((QRegExp(r"\b@\w+\b"),
                "decorator"))
        stringRe = QRegExp(r"""(?:'[^']*'|"[^"]*")""")
        stringRe.setMinimal(True)
        AssemblyHighlighter.Rules.append((stringRe, "string"))
        self.stringRe = QRegExp(r"""(:?"["]".*"["]"|'''.*''')""")
        self.stringRe.setMinimal(True)
        AssemblyHighlighter.Rules.append((self.stringRe, "string"))
        self.tripleSingleRe = QRegExp(r"""'''(?!")""")
        self.tripleDoubleRe = QRegExp(r'''"""(?!')''')


    @staticmethod
    def initializeFormats():
        baseFormat = QTextCharFormat()
        baseFormat.setFontFamily("courier")
        baseFormat.setFontPointSize(12)
        for name, color in (("normal", Qt.black),
                ("keyword", Qt.darkBlue), ("builtin", Qt.darkRed),
                ("constant", Qt.darkGreen),
                ("decorator", Qt.darkBlue), ("comment", Qt.darkGreen),
                ("string", Qt.darkYellow), ("number", Qt.darkMagenta),
                ("error", Qt.darkRed), ("pyqt", Qt.darkCyan)):
            format = QTextCharFormat(baseFormat)
            format.setForeground(QColor(color))
            if name in ("keyword", "decorator"):
                format.setFontWeight(QFont.Bold)
            if name == "comment":
                format.setFontItalic(True)
            AssemblyHighlighter.Formats[name] = format


    def highlightBlock(self, text):
        NORMAL, TRIPLESINGLE, TRIPLEDOUBLE, ERROR = range(4)

        textLength = len(text)
        prevState = self.previousBlockState()

        self.setFormat(0, textLength,
                       AssemblyHighlighter.Formats["normal"])

        if text.startswith("Traceback") or text.startswith("Error: "):
            self.setCurrentBlockState(ERROR)
            self.setFormat(0, textLength,
                           AssemblyHighlighter.Formats["error"])
            return
        if (prevState == ERROR and
            not (text.startswith(sys.ps1) or text.startswith(";"))):
            self.setCurrentBlockState(ERROR)
            self.setFormat(0, textLength,
                           AssemblyHighlighter.Formats["error"])
            return

        for regex, format in AssemblyHighlighter.Rules:
            i = regex.indexIn(text)
            while i >= 0:
                length = regex.matchedLength()
                self.setFormat(i, length,
                               AssemblyHighlighter.Formats[format])
                i = regex.indexIn(text, i + length)

        # Slow but good quality highlighting for comments. For more
        # speed, comment this out and add the following to __init__:
        # AssemblyHighlighter.Rules.append((QRegExp(r"#.*"), "comment"))
        if not text:
            pass
        elif text[0] == ';':
            self.setFormat(0, len(text),
                           AssemblyHighlighter.Formats["comment"])
        else:
            stack = []
            for i, c in enumerate(text):
                if c in ('"', "'"):
                    if stack and stack[-1] == c:
                        stack.pop()
                    else:
                        stack.append(c)
                elif c == ";" and len(stack) == 0:
                    self.setFormat(i, len(text),
                                   AssemblyHighlighter.Formats["comment"])
                    break

        self.setCurrentBlockState(NORMAL)

        if self.stringRe.indexIn(text) != -1:
            return
        # This is fooled by triple quotes inside single quoted strings
        for i, state in ((self.tripleSingleRe.indexIn(text),
                          TRIPLESINGLE),
                         (self.tripleDoubleRe.indexIn(text),
                          TRIPLEDOUBLE)):
            if self.previousBlockState() == state:
                if i == -1:
                    i = text.length()
                    self.setCurrentBlockState(state)
                self.setFormat(0, i + 3,
                               AssemblyHighlighter.Formats["string"])
            elif i > -1:
                self.setCurrentBlockState(state)
                self.setFormat(i, text.length(),
                               AssemblyHighlighter.Formats["string"])

'''
    def rehighlight(self):
        QApplication.setOverrideCursor(QCursor(
                                                    Qt.WaitCursor))
        QSyntaxHighlighter.rehighlight(self)
        QApplication.restoreOverrideCursor()
'''

class TextEdit(QTextEdit):

    def __init__(self, parent=None):
        super(TextEdit, self).__init__(parent)


    def event(self, event):
        if (event.type() == QEvent.KeyPress and
            event.key() == Qt.Key_Tab):
            cursor = self.textCursor()
            cursor.insertText("    ")
            return True
        return QTextEdit.event(self, event)


class MainWindow(QMainWindow):

    def __init__(self, filename=None, parent=None):
        super(MainWindow, self).__init__(parent)

        font = QFont("Courier", 11)
        self.toolbar = QToolBar()
        font.setFixedPitch(True)
        self.editor = TextEdit()
        self.editor.setFont(font)
        self.highlighter = AssemblyHighlighter(self.editor.document())
        self.setCentralWidget(self.editor)

        status = self.statusBar()
        status.setSizeGripEnabled(False)
        status.showMessage("Ready", 5000)

        fileNewAction = self.createAction("&New...", self.fileNew,
                QKeySequence.New, "filenew", "Create a Assembly file")
        fileOpenAction = self.createAction("&Open...", self.fileOpen,
                QKeySequence.Open, "fileopen",
                "Open an existing Assembly file")
        self.fileSaveAction = self.createAction("&Save", self.fileSave,
                QKeySequence.Save, "filesave", "Save the file")
        self.fileSaveAsAction = self.createAction("Save &As...",
                self.fileSaveAs, icon="filesaveas",
                tip="Save the file using a new name")
        self.connect_boardAction = self.createAction("&connect JTTAG",self.connect_board,
                                                     icon = 'connect',tip = "connect the jttag")

        fileQuitAction = self.createAction("&Quit", self.close,
                "Ctrl+Q", "filequit", "Close the application")
        self.editCopyAction = self.createAction("&Copy",
                self.editor.copy, QKeySequence.Copy, "editcopy",
                "Copy text to the clipboard")
        self.editCutAction = self.createAction("Cu&t", self.editor.cut,
                QKeySequence.Cut, "editcut",
                "Cut text to the clipboard")
        self.editPasteAction = self.createAction("&Paste",
                self.editor.paste, QKeySequence.Paste, "editpaste",
                "Paste in the clipboard's text")
        self.change_binAction = self.createAction("&change into binary",
                self.change_bin, "Ctrl+]", "change_bin",
                "change asm into binary")
        self.change_coeAction = self.createAction("&change into coe",
                                                  self.change_coe, "Ctrl+e", "change_coe",
                                                  "change asm into coe")
        self.change_hexAction = self.createAction("&change into hex",
                self.change_hex, "Ctrl+[", "change_hex",
                "change asm into hex")
        self.editIndentAction = self.createAction("&Indent",
                self.editIndent, "Ctrl+u", "editindent",
                "Indent the current line or selection")
        self.editUnindentAction = self.createAction("&Unindent",
                self.editUnindent, "Ctrl+i", "editunindent",
                "Unindent the current line or selection")

        fileMenu = self.menuBar().addMenu("&File")
        self.addActions(fileMenu, (fileNewAction, fileOpenAction,
                self.fileSaveAction, self.fileSaveAsAction,self.connect_boardAction, None,
                fileQuitAction,))
        editMenu = self.menuBar().addMenu("&Edit")
        self.addActions(editMenu, (self.editCopyAction,
                self.editCutAction, self.editPasteAction, None,
                self.change_binAction, self.change_hexAction,
                                   self.editIndentAction, self.editUnindentAction))
        fileToolbar = self.addToolBar("File")
        fileToolbar.setObjectName("FileToolBar")
        self.addActions(fileToolbar, (fileNewAction,
                                      fileOpenAction,
                                      self.fileSaveAction,
                                      self.connect_boardAction))

        #editToolbar = self.addToolBar("Edit")
        editToolbar = self.addToolBar(Qt.LeftToolBarArea,self.toolbar)
        #editToolbar.setObjectName("EditToolBar")
        self.toolbar.addActions( (self.editCopyAction,
                self.editCutAction, self.editPasteAction, None,
                self.change_binAction,
                                  self.change_hexAction,self.change_coeAction,

                                  self.editIndentAction, self.editUnindentAction))
        #editToolbar = self.addToolBar(Qt.LeftToolBarArea,QToolBar())



        self.editor.selectionChanged.connect(self.updateUi)
        self.editor.document().modificationChanged.connect(self.updateUi)
        QApplication.clipboard().dataChanged.connect(self.updateUi)

        self.resize(800, 600)
        self.setWindowTitle("PRV332 ide")
        self.filename = filename
        self.bin_name = None
        if self.filename is not None:
            self.loadFile()
        self.updateUi()


    def updateUi(self, arg=None):
        self.fileSaveAction.setEnabled(
                self.editor.document().isModified())
        enable = not self.editor.document().isEmpty()
        self.fileSaveAsAction.setEnabled(enable)
        self.connect_boardAction.setEnabled(enable)
        self.change_binAction.setEnabled(enable)
        self.change_hexAction.setEnabled(enable)
        self.change_coeAction.setEnabled(enable)
        self.editIndentAction.setEnabled(enable)
        self.editUnindentAction.setEnabled(enable)
        enable = self.editor.textCursor().hasSelection()
        self.editCopyAction.setEnabled(enable)
        self.editCutAction.setEnabled(enable)
        self.editPasteAction.setEnabled(self.editor.canPaste())


    def createAction(self, text, slot=None, shortcut=None, icon=None,
                     tip=None, checkable=False, signal="triggered()"):
        action = QAction(text, self)
        if icon is not None:
            action.setIcon(QIcon("{0}.ico".format(icon)))
        if shortcut is not None:
            action.setShortcut(shortcut)
        if tip is not None:
            action.setToolTip(tip)
            action.setStatusTip(tip)
        if slot is not None:
            action.triggered.connect(slot)
        if checkable:
            action.setCheckable(True)
        return action


    def addActions(self, target, actions):
        for action in actions:
            if action is None:
                target.addSeparator()
            else:
                target.addAction(action)


    def closeEvent(self, event):
        if not self.okToContinue():
            event.ignore()


    def okToContinue(self):
        if self.editor.document().isModified():
            reply = QMessageBox.question(self,
                            "PRV332 ide - Unsaved Changes",
                            "Save unsaved changes?",
                            QMessageBox.Yes|QMessageBox.No|
                            QMessageBox.Cancel)
            if reply == QMessageBox.Cancel:
                return False
            elif reply == QMessageBox.Yes:
                return self.fileSave()
        return True


    def fileNew(self):
        if not self.okToContinue():
            return
        document = self.editor.document()
        document.clear()
        document.setModified(False)
        self.filename = None
        self.setWindowTitle("PRV332 ide - Unnamed")
        self.updateUi()


    def fileOpen(self):
        if not self.okToContinue():
            return
        dir = (os.path.dirname(self.filename)
               if self.filename is not None else ".")
        fname = str(QFileDialog.getOpenFileName(self,
                "PRV332 ide - Choose File", dir,
                "Assembly language sourse file (*.asm)")[0])
        if fname:
            self.filename = fname
            self.loadFile()


    def loadFile(self):
        fh = None
        try:
            fh = QFile(self.filename)
            if not fh.open(QIODevice.ReadOnly):
                raise IOError(str(fh.errorString()))
            stream = QTextStream(fh)
            stream.setCodec("UTF-8")

            self.editor.setPlainText(stream.readAll())
            self.editor.document().setModified(False)
        except EnvironmentError as e:
            QMessageBox.warning(self, "PRV332 ide -- Load Error",
                    "Failed to load {0}: {1}".format(self.filename, e))
        finally:
            if fh is not None:
                fh.close()
        self.setWindowTitle("PRV332 ide - {0}".format(
                QFileInfo(self.filename).fileName()))


    def fileSave(self):
        if self.filename is None:
            return self.fileSaveAs()
        fh = None
        try:
            fh = QFile(self.filename)
            #print(fh)
            if not fh.open(QIODevice.WriteOnly):
                raise IOError(str(fh.errorString()))
            stream = QTextStream(fh)
            stream.setCodec("UTF-8")
            stream << self.editor.toPlainText()

            self.editor.document().setModified(False)
        except EnvironmentError as e:
            QMessageBox.warning(self, "PRV332 ide -- Save Error",
                    "Failed to save {0}: {1}".format(self.filename, e))
            return False
        finally:
            if fh is not None:
                fh.close()
        return True


    def fileSaveAs(self):
        filename = self.filename if self.filename is not None else "."
        filename,filetype = QFileDialog.getSaveFileName(self,
                "PRV332 ide -- Save File As", filename,
                "Assembly language sourse file (*.asm)")
        if filename:
            self.filename = filename
            self.setWindowTitle("PRV332 ide - {0}".format(
                    QFileInfo(self.filename).fileName()))
            return self.fileSave()
        return False
    def run_socket(self):
        #os.system("python t_server.py")
        os.system("t_server.exe")
    def connect_board(self):
        threadsockct = Thread(target=self.run_socket)
        threadsockct.start()



    def bin_fileSave(self):
        line0 = 0
        err = 0
        try:

            fname = QFileDialog.getSaveFileName(self, "Write File", "./", "All (*.*)")  # 写入文件首先获取文件路径
        except:
            err = 1
        if fname[0]:  # 如果获取的路径非空
            with open('asm.txt',"r") as ass_file:
                with open(fname[0],"w+") as bin_file:
                    for line in ass_file:
                        line0+=1
                        try:
                            changed = change_into_bin(line)
                            chan = changed.after_change_str + '\n'
                            bin_file.write(chan)

                        except:
                            if(not (line == '\n') ):
                                QMessageBox.critical(self, "错误", "第"+str(line0)+"行出错未能生成bin\n"+line)
                            err = 1
        if not err:
            QMessageBox.about(self, "成功",
                          "成功生成bin于\n"+fname[0])




    def change_bin(self):

        te =self.editor.toPlainText()
        with open('asm.txt', 'w+') as assfile:
            assfile.write(te)
        #with open('ass.txt', 'r+') as assfile:
        #   with open('bin.txt', 'w+') as binfile:
        #       for line in assfile:
        #           changed = change_into_bin(line)

        #           chan = changed.after_change_str + '\n'
        #           binfile.write(chan)
        self.bin_fileSave()


        '''
        这一段原来是作为一个indent
        但是后来觉得可能不大会用到
        '''
    def coe_fileSave(self):
        line0 = 0
        err = 0
        #c = self.count
        try:
            fname = QFileDialog.getSaveFileName(self, "Write File", "./", "All (*.*)")  # 写入文件首先获取文件路径
        except:
            err = 1
        if fname[0]:  # 如果获取的路径非空
            with open('asm.txt', "r") as ass_file:
                count = 0
                #f = open("filepath", "r")

                with open(fname[0], "w+") as coe_file:
                    coe_file.write(';this coe file is made by prv332 ide \n;if you have any questions you can e-mail chen.yuheng@nexuslink.cn\n')
                    coe_file.write('memory_initialization_radix = 2;\nmemory_initialization_vector =\n')
                    for line in ass_file:
                        line0 += 1
                        try:
                            changed = change_into_bin(line)
                            if line0 == self.count:
                                chan = changed.after_change_str + ';' + '\n'
                            else:
                                chan = changed.after_change_str +','+ '\n'
                            coe_file.write(chan)

                        except:
                            if (not (line == '\n')):
                                QMessageBox.critical(self, "错误", "第" + str(line0) + "行出错未能生成coe\n" + line)
                            err = 1
        if not err:
            QMessageBox.about(self, "成功",
                              "成功生成coe于\n" + fname[0])


    def change_coe(self):
        te = self.editor.toPlainText()
        with open('asm.txt','w+') as assfile:
            assfile.write(te)
        self.count = 0
        for index, line in enumerate(open('asm.txt', 'r')):
            self.count += 1
        print(self.count)
        self.coe_fileSave()
    def editIndent(self):
        cursor = self.editor.textCursor()
        cursor.beginEditBlock()
        if cursor.hasSelection():
            start = pos = cursor.anchor()
            end = cursor.position()
            if start > end:
                start, end = end, start
                pos = start
            cursor.clearSelection()
            cursor.setPosition(pos)
            cursor.movePosition(QTextCursor.StartOfLine)
            while pos <= end:
                cursor.insertText("    ")
                cursor.movePosition(QTextCursor.Down)
                cursor.movePosition(QTextCursor.StartOfLine)
                pos = cursor.position()
            cursor.setPosition(start)
            cursor.movePosition(QTextCursor.NextCharacter,
                                QTextCursor.KeepAnchor, end - start)
        else:
            pos = cursor.position()
            cursor.movePosition(QTextCursor.StartOfBlock)
            cursor.insertText("    ")
            cursor.setPosition(pos + 4)
        cursor.endEditBlock()
    def hex_fileSave(self):
        err = 0
        try:
            fname = QFileDialog.getSaveFileName(self, "Write File", "./", "All (*.*)")  # 写入文件首先获取文件路径
        except:
            err = 1
        if fname[0]:  # 如果获取的路径非空
            #a = change_into_hex('asm.txt', fname[0])

            try:
                a = change_into_hex('asm.txt', fname[0])
            except:

                QMessageBox.critical(self, "错误", "未能生成hex\n")
                err = 1
        if not err:
            a.write_hex()
            QMessageBox.about(self, "成功",
                              "成功生成hex于\n" + fname[0])
    '''
    from_file = 'asm.txt'
    filea = 'hex.txt'
    a = change_into_hex(from_file, filea)
    print('地址.......', a.location_dict)

    print('global_label_data_dict', global_label_data_dict)
    print('end')
    a.write_hex()
    '''




    def change_hex(self):
        '''
        from_file = 'asm.txt'
        filea = 'hex.txt'
        a = change_into_hex(from_file,filea)
        print('地址',a.location_dict)
        print('end')
        a.write_hex()
        '''
        te = self.editor.toPlainText()
        with open('asm.txt', 'w+') as assfile:
            assfile.write(te)
        self.hex_fileSave()



    def editUnindent(self):
        cursor = self.editor.textCursor()
        cursor.beginEditBlock()
        if cursor.hasSelection():
            start = pos = cursor.anchor()
            end = cursor.position()
            if start > end:
                start, end = end, start
                pos = start
            cursor.setPosition(pos)
            cursor.movePosition(QTextCursor.StartOfLine)
            while pos <= end:
                cursor.clearSelection()
                cursor.movePosition(QTextCursor.NextCharacter,
                                    QTextCursor.KeepAnchor, 4)
                if cursor.selectedText() == "    ":
                    cursor.removeSelectedText()
                cursor.movePosition(QTextCursor.Down)
                cursor.movePosition(QTextCursor.StartOfLine)
                pos = cursor.position()
            cursor.setPosition(start)
            cursor.movePosition(QTextCursor.NextCharacter,
                                QTextCursor.KeepAnchor, end - start)
        else:
            cursor.clearSelection()
            cursor.movePosition(QTextCursor.StartOfBlock)
            cursor.movePosition(QTextCursor.NextCharacter,
                                QTextCursor.KeepAnchor, 4)
            if cursor.selectedText() == "    ":
                cursor.removeSelectedText()
        cursor.endEditBlock()


def main():
    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon("p1.ico"))
    fname = None
    if len(sys.argv) > 1:
        fname = sys.argv[1]
    form = MainWindow(fname)
    form.show()
    app.exec_()
'''
thread_01 = Thread(target=show_with_hdmi)
thread_02 = Thread(target = thread2)
print('ok')
thread_01.start()
thread_02.start()
'''

thread_main = Thread(target = main())
thread_main.start()





