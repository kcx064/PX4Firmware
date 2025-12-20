#echo
echo "开始备份项目文件..."
zip -r ../PX4Firmware-1.15.4.zip . -x "build/*"
echo "压缩完成，正在复制到目标位置..."

cp ../PX4Firmware-1.15.4.zip /mnt/d/myWorkSpace/PostDoctor
echo "备份已成功保存到/mnt/d/myWorkSpace/PostDoctor"
