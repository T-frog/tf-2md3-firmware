コンパイル方法
筑波大学 渡辺敦志

■ 環境
 Linux環境を想定(gcc,makeが用意できれば他の環境でもOK)
 ・GNU make
 ・arm-elf-gcc
　　- 渡辺はgccのソースコードからarm-elf用にコンパイルしてコンパイラを生成
　　- おそらく、出来合いのコンパイラでもOK(未確認) http://www.gnuarm.com/
 書き込みには、Atmel社 SAM-BA が必要。
  http://www.atmel.com/dyn/products/tools_card.asp?tool_id=3883

■ コンパイル・書き込み方法
1. tfrog-motordriver ディレクトリ内でmakeコマンド
2. tfrog-motordriver/bin 内で
	% samba -i /dev/ttyACM0 (Linuxの場合)
	samba> flash 0x00100000 cdc-test-theva-rev1-at91sam7se512-flash.bin
	samba> nvm 4


■ ライセンス
・同梱の at91lib のオープンソースライセンスを継承するはず

