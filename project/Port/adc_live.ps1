# =============================================================================
#  adc_live.ps1  -- 实时监视 ADC2_IN15 (PB15) 上的输入信号
#
#  为什么需要它：
#    跟踪器读到的永远是 0，但"线到底接上没有"从代码里看不出来。
#    这个脚本每秒读几次 ADC 的 DMA 缓冲，直接显示 PB15 上的电压范围。
#
#  怎么用：
#    1) 关掉 RTT 桥（J-Link 独占），直接双击 adc_live.bat
#    2) 看输出的 pk 值：
#         pk ≈ 4000  -> 有完整信号，正常
#         pk ≈ 0     -> 引脚上是一条直线（没接信号 / 接错脚 / 线断了）
#       电压列直接告诉你现在引脚上是多少伏
#    3) 一边跑一边动那根线，看数字变化，就能定位问题
#
#  判定方法：
#    把 PB15 用杜邦线短到 3V3  -> max 应该跳到 4095（3.3V）
#    再短到 GND               -> min 应该变成 0
#    两个都能变 = ADC + 引脚都好，问题在信号源或线上
#    怎么都不变 = 引脚/焊接有问题
#
#  注意：脚本会把当前 ADC 缓冲原样读出来，不做任何配置修改，只读不写。
# =============================================================================

$JLINK = "D:\MY_tool_at_ALL\J_Flash_APP\J-linkV9.64SEGGER\JLink_V964\JLink.exe"
$BUF_ADDR = 0x20001654      # adc_buf（从 DMA1_Channel4->CMAR 读出来的，和 nm 一致）
$BUF_LEN  = 2048            # ADC_BUF_LEN
$VREF     = 3.3

if (-not (Test-Path $JLINK)) { Write-Host "找不到 JLink.exe: $JLINK" -ForegroundColor Red; exit 1 }

$scr = Join-Path $env:TEMP "adc_live_monitor.jlink"
$lines = @(
    "si SWD",
    "speed 4000",
    "device STM32G431C8",
    "connect",
    ("mem16 0x{0:X}, {1}" -f $BUF_ADDR, $BUF_LEN),
    "qc"
)
$lines | Set-Content -Path $scr -Encoding ASCII

Write-Host ""
Write-Host "=== PB15 (ADC2_IN15) 输入信号实时监视 ===" -ForegroundColor Cyan
Write-Host "  pk 接近 4000 = 有信号；pk 接近 0 = 引脚上是一条直线" -ForegroundColor Cyan
Write-Host "  按 Ctrl+C 退出" -ForegroundColor Cyan
Write-Host ""

try {
    while ($true) {
        $out = & $JLINK -CommanderScript $scr 2>&1
        $vals = New-Object System.Collections.Generic.List[int]

        foreach ($l in $out) {
            $m = [regex]::Match($l, '^\s*([0-9A-F]{8})\s*=\s*(.+)$')
            if (-not $m.Success) { continue }
            $addr = [Convert]::ToUInt32($m.Groups[1].Value, 16)
            $i = 0
            foreach ($v in ($m.Groups[2].Value.Trim() -split '\s+')) {
                if ($v -match '^[0-9A-F]{4}$') {
                    # 只接受落在 adc_buf 范围内的样本，防止 J-Link 多读出来的
                    # 邻居变量(tx_buf 等)污染 min/max
                    $a = $addr + 2 * $i
                    if (($a -ge $BUF_ADDR) -and ($a -lt ($BUF_ADDR + 2 * $BUF_LEN))) {
                        $vals.Add([Convert]::ToInt32($v, 16))
                    }
                    $i++
                }
            }
        }

        if ($vals.Count -gt 100) {
            $mn = ($vals | Measure-Object -Minimum).Minimum
            $mx = ($vals | Measure-Object -Maximum).Maximum
            $pk = $mx - $mn
            $vlo = [math]::Round($mn * $VREF / 4095.0, 3)
            $vhi = [math]::Round($mx * $VREF / 4095.0, 3)

            $color = if ($pk -gt 500) { "Green" } else { "Yellow" }
            Write-Host ("  min={0,4} ({1,5} V)   max={2,4} ({3,5} V)   pk={4,4}   [{5} 点]" -f `
                        $mn, $vlo, $mx, $vhi, $pk, $vals.Count) -ForegroundColor $color
        } else {
            Write-Host "  读取失败（J-Link 是否被 RTT 桥占用？先关掉桥）" -ForegroundColor Red
        }

        Start-Sleep -Milliseconds 200
    }
} finally {
    Remove-Item $scr -Force -ErrorAction SilentlyContinue
}
