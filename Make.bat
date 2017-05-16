
@IF "%1"=="" GOTO NO_PARAM
@IF "%1"=="-b" GOTO BUILD
@IF "%1"=="-u" GOTO UPLOAD
@IF "%1"=="-m" GOTO BUILD
echo %1
@GOTO WRONG_PARAM

:BUILD

set "origin=%2%"

@del /q C:\RB3rt\sdk\workspace\app
@del /q C:\RB3rt\sdk\workspace\project\*

@xcopy /y/q %origin%\* C:\RB3rt\sdk\workspace\project

@c:\cygwin64\bin\bash.exe -l -c " pwd ; cd /cygdrive/c/RB3rt/sdk/workspace ; pwd ; make app=project"
xcopy /y C:\RB3rt\sdk\workspace\app %origin%\.

@IF "%1"=="-m" GOTO UPLOAD
@GOTO END


:UPLOAD
@c:\cygwin64\bin\bash.exe -l -c " pwd ; cd /cygdrive/c/RB3rt/sdk/workspace ; pwd ; make upload"

@GOTO END


:NO_PARAM
@echo "%0: no parameters"
@GOTO END

:WRONG_PARAM
@echo "%0: wrong parameters"
:END