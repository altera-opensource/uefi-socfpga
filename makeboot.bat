@echo off
call setup
if "%1"=="clean" GOTO MAKE_CLEAN
if "%1"=="all" GOTO MAKE_ALL
GOTO MAKE_ALL
GOTO DONE

:MAKE_ALL
make %2 %3 %4 %5 %6 %7 %8 %9
GOTO DONE

:MAKE_CLEAN
make clean
GOTO DONE

:DONE