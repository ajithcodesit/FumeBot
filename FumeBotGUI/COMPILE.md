## **Compiling UI and Resource Files**

For compiling the resource file containing the icons and images to be imported into the compiled UI files use the following command:
```bash
pyrcc4 -py3 FumeBotImages.qrc -o FumeBotImages_rc.py
```
After the above resource file is compiled, the UI files can be compiled to python classes that can be used in the GUI application using the following commands:

```bash
pyuic4 --from-imports -x FumeBot_UI.ui -o FumeBot_UI.py
pyuic4 --from-imports -x FumeBotAbout_UI.ui -o FumeBotAbout_UI.py
```
This requires PyQt4 to be installed in Windows 10 and the site package file to be in the environment variables for easy access to `pyrcc4` and `pyuic4`.