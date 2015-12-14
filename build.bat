@echo off

rem clean up any files / folders
if not exist output mkdir output
del /S /Q output
for /D %%p in ("output\*") do rmdir "%%p" /s /q


rem build 3D library
msbuild source\Jitter.sln /p:Configuration=Release /t:Rebuild

rem ready for packaging
if not exist output\net4 mkdir output\net4
if not exist output\Portable mkdir output\Portable
copy source\Jitter\bin\net4\Release\Jitter.dll output\net4\
copy source\Jitter\bin\net4\Release\Jitter.pdb output\net4\
copy source\Jitter\bin\net4\Release\Jitter.xml output\net4\
copy source\Jitter\bin\Portable\Release\Jitter.dll output\Portable\
copy source\Jitter\bin\Portable\Release\Jitter.pdb output\Portable\
copy source\Jitter\bin\Portable\Release\Jitter.xml output\Portable\

rem package
nuget pack nuget\Jitter.nuspec -OutputDirectory output

rem package
xamarin-component package ./component/
mv component\*.xam output
