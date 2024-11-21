#!/usr/bin/env bash

function check_command() {
   local missing_tools=()
   
   if ! command -v parallel &> /dev/null; then
      missing_tools+=("parallel (sudo apt-get install parallel)")
   fi
   if ! command -v pngcheck &> /dev/null; then
      missing_tools+=("pngcheck (sudo apt-get install pngcheck)")
   fi
   if ! command -v pngfix &> /dev/null; then
      missing_tools+=("pngfix (sudo apt-get install libpng-tools)")
   fi
   if ! command -v convert &> /dev/null; then
      missing_tools+=("convert (sudo apt-get install imagemagick)")
   fi
   if ! command -v djpeg &> /dev/null || ! command -v pnmtopng &> /dev/null; then
      missing_tools+=("djpeg and pnmtopng (sudo apt-get install netpbm libjpeg-progs)")
   fi
   if ! command -v ffmpeg &> /dev/null; then
      missing_tools+=("ffmpeg (sudo apt-get install ffmpeg)")
   fi

   if [ ${#missing_tools[@]} -ne 0 ]; then
      echo "缺少必要的工具，请安装："
      printf '%s\n' "${missing_tools[@]}"
      exit 1
   fi
}

function check_png() {
   pngcheck "$1" &> /dev/null
   return $?
}

function repair_single_png() {
   local file="$1"
   
   # 只有当pngcheck检测到错误时才修复
   if ! check_png "$file"; then
      echo "发现损坏的PNG文件: $file"
      
      # 方法1：使用pngfix
      echo "尝试使用pngfix修复 $file..."
      if pngfix --quiet --out="${file}.fixed" "$file" && [ -f "${file}.fixed" ]; then
         mv "${file}.fixed" "$file"
         echo "使用pngfix成功修复 $file"
         return 0
      fi
      
      # 方法2：使用ImageMagick
      echo "pngfix失败，尝试使用ImageMagick修复 $file..."
      if convert "$file" "${file}.converted" 2>/dev/null && [ -f "${file}.converted" ]; then
         mv "${file}.converted" "$file"
         echo "使用ImageMagick成功修复 $file"
         return 0
      fi
      rm -f "${file}.converted"
      
      # 方法3：使用djpeg和pnmtopng
      echo "ImageMagick失败，尝试使用djpeg修复 $file..."
      if djpeg -pnm "$file" > "${file}.pnm" 2>/dev/null && [ -f "${file}.pnm" ]; then
         if pnmtopng "${file}.pnm" > "${file}.new" 2>/dev/null && [ -f "${file}.new" ]; then
            mv "${file}.new" "$file"
            echo "使用djpeg成功修复 $file"
            rm -f "${file}.pnm"
            return 0
         fi
         rm -f "${file}.new"
      fi
      rm -f "${file}.pnm"
      
      # 方法4：使用ffmpeg
      echo "djpeg失败，尝试使用ffmpeg修复 $file..."
      if ffmpeg -y -i "$file" -c:v png "${file}.ffmpeg" 2>/dev/null && [ -f "${file}.ffmpeg" ]; then
         mv "${file}.ffmpeg" "$file"
         echo "使用ffmpeg成功修复 $file"
         return 0
      fi
      rm -f "${file}.ffmpeg"
      
      echo "所有修复方法都失败了: $file"
      # 清理可能存在的临时文件
      rm -f "${file}.fixed" "${file}.converted" "${file}.new" "${file}.pnm" "${file}.ffmpeg"
   fi
}

export -f check_png repair_single_png

function repair_png() {
   if [ -z "$1" ]; then
      echo "Usage: $0 <directory>"
      exit 1
   fi

   # 检查必要的命令
   check_command

   # 使用parallel并行处理所有PNG文件
   find "$1" -name "*.png" | parallel --bar repair_single_png {}
}

repair_png "$@"
