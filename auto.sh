#!/bin/sh

PRG=$0
usage()
{
    echo "usage: auto [<args>]" 
    echo ""
    echo "The most commonly used commands are:"
    echo "  generate_debs   Generates debian on 32 or 64 bit machines"
    echo "  generate_docs   Generates website documentation"
    echo "  upload_web      Uploads the new website"
    echo ""
    exit 1
}

debs()
{
mac=$(echo `uname -m`)
echo ${mac}
if [ ${mac} = "x86_64" ]; then
  echo "Generating 64 bit deb"
  cmake -D CPACK_SYSTEM_NAME=amd64 .
  cpack -G DEB
else
  echo "Generating 32 bit deb"
  cmake -D CPACK_SYSTEM_NAME=i386 .
  cpack -G DEB
fi
}

web()
{
scp *.deb pushkar7@golems.org:~/dart.golems.org/downloads/
cd docs
mv html grip
tar czf grip.tar.gz grip
scp grip.tar.gz pushkar7@golems.org:~/dart.golems.org/
ssh pushkar7@golems.org 'cd dart.golems.org; tar -xvf grip.tar.gz'
}

[ "$#" -lt 1 ] && usage

# parse commandline
while [ $# -gt 0 ]
do
      arg="$1"
      case "$arg" in
         generate_debs)
         debs    
         ;;
         generate_docs)
         doxygen
         ;;
         upload_web)
         web
         ;;
         --) shift; break;;  # no more options
         -*) usage ;; 
         *) break;; # not option, its some argument
       esac
       shift
done
