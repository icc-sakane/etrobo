#####################################################################################
#
#     Lコース用
#
#####################################################################################

include EV3RT_TECS

#センサー・モーターのポート設定
TOUCH_SENSOR = :port_1
COLOR_SENSOR = :port_3
GYRO_SENSOR  = :port_4
SONAR_SENSOR = :port_2

TAIL_MOTOR   = :port_a
RIGHT_MOTOR  = :port_b
LEFT_MOTOR   = :port_c

#灰色探知用
GRAY_RANGE = 10 #探索範囲
gray_arr = []
gray_arr2 = []

class Array
  # 要素をto_iした値の平均を算出する
  def avg
    inject(0.0){|r,i| r+=i.to_i }/size
  end
  # 要素をto_iした値の分散を算出する
  def variance
    a = avg
    inject(0.0){|r,i| r+=(i.to_i-a)**2 }/size
  end
	def standard
		variance**(1/2.0)
	end
end

#??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽL??ｿｽ?ｿｽﾌ定数??ｿｽ?ｿｽl??ｿｽ?ｿｽﾍ個托ｿｽ/??ｿｽ?ｿｽﾂ具ｿｽ??ｿｽ?ｿｽﾉ搾ｿｽ??ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ??ｿｽ?ｿｽﾄ変更??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽK??ｿｽ?ｿｽv??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽﾜゑｿｽ
gyro_offset = 1		  	#ロボット(固有名)のジャイロセンサー固有基礎設定値
SONAR_ALERT_DISTANCE = 30 #ソナーアラート値[cm]
TAIL_ANGLE_STAND_UP = 91  #ロボットを立った状態にするためのテールの角度の初期値[x]
TAIL_ANGLE_DRIVE = 3      #[x]
TAIL_ANGLE_TAIL_DRIVE = 75 #tail_drive
P_GAIN = 1.5              # ??ｿｽ?ｿ?ｽ?ｿｽ?ｿｽ??ｿｽ?ｿｽS??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ~??ｿｽ?ｿｽp??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ[??ｿｽ?ｿｽ^??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽW??ｿｽ?ｿｽ??ｿｽ?ｿｽ
PWM_ABS_MAX = 60          # ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽS??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ~??ｿｽ?ｿｽp??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ[??ｿｽ?ｿｽ^??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽPWM??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽﾎ最托ｿｽ??ｿｽ?ｿｽl


# PID制御に関わる定数をセット
# 周期を設定
PERIOD = 0.004
#PDI制御(フィードバック制御)初期値
KP = 1.7	#比例ゲイン初期値
KI = 0		#積分ゲイン初期値
KD = 0.23	#微分ゲイン初期値

iremono = [
  #0.21
# => [motor_w_count,lm,rm,KP,KI,KD,i,threshold,gyro_offset]	-：黒寄り
    [    0, 1, 1,1,0.162, 0.04, 20,  0, 1], # pid0
#    [ 1000, 1, 1,1,0.162, 0.04, 20,  -2, 1], # pid1(1st_straight)
    [999999, 1, 1,0.996,0.139,0.034,  20, 0,  1], # pid2(1carb)
    [ 10000, 1, 1,0.996,0.139,0.034,  75, -3,  2], # pid3(1carb)
    [ 13000, 1, 1,0.126,0.162, 0.04, 75,  10,  1], # pid4(2st_straight)
    [14000, 1, 1, 0.696,0.225,0.056,  100,  5, 0], # pid5(2carb_mid)
    [19000, 1, 1, 0.696,0.176,0.044,  60, -3,  0], # pid6(2carb_2)
    [20000, 1, 1, 0.996,0.176,0.044,  60, -3,  -1], #pid7
    [24500, 1, 1,0.2,0.162, 0.034,  100,  3,  1], #pid8(3st_straight)
    [26000, 1, 1,0.2,0.162, 0.034, 100,  3,  1], # pid9(3st_straight)
    [28000, 1, 1, 0.996, 0.17,0.042,  60,  -3,  0], # pid10(3carb)
    [30000, 1, 1,0.126,0.162, 0.04,  50, 0,  0], # pid11(4st_straight_goal)
    [99999, 1, 1,0.126,0.162, 0.04,  50,  0,  0], # 4st_straight
    [999999] #end
]

$diff = Array.new(2) # ??ｿｽ?ｿｽZ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽT??ｿｽ?ｿｽ[??ｿｽ?ｿｽl??ｿｽ?ｿｽﾛ托ｿｽ??ｿｽ?ｿｽp??ｿｽ?ｿｽﾌ変撰ｿｽ
$integral = 0# ??ｿｽ?ｿｽﾏ包ｿｽ??ｿｽ?ｿｽv??ｿｽ?ｿｽZ??ｿｽ?ｿｽp??ｿｽ?ｿｽﾌ変撰ｿｽ

#*****************************************************************************
# カラーセンサーのキャリブレーション
# 引数 : n (測定回数) 省略時は10
# n回測定した平均値
#*****************************************************************************
def color_calibration(n=10)
    loop {
        break if $touch.pressed?
        RTOS.delay(10)
    }
    col = 0
    n.times { col += $color.reflect}
    col = (col / n).round
    Speaker.tone(:a5, 200)
    RTOS.delay(500)
    col

end
#*****************************************************************************
# 走行体完全停止用モータの角度制御
# 引数 : angle (モータ目標角度[度])
# 返り値 : 無し
#*****************************************************************************
def tail_control(angle)
    pwm = ((angle - $motor_t.count) * P_GAIN).to_i
    pwm = (pwm > PWM_ABS_MAX)  ? PWM_ABS_MAX :
    (pwm < -PWM_ABS_MAX) ? -PWM_ABS_MAX : pwm
    $motor_t.power = pwm
    $motor_t.stop(true) if pwm == 0
end
# PID制御
def pid_control(target_val,sensor_val )

    $diff[0] = $diff[1]
    $diff[1] = sensor_val - target_val
    $integral += (($diff[1
] || 0 ) + ($diff[0] || 0)) / 2.0 * PERIOD

    prp = KP * $diff[1]
    ing = KI * $integral
    dif = KD * (($diff[1] || 0) - ($diff[0] || 0)) / PERIOD

    #100以上出さない
    return output_limit(prp + ing + dif)
end

# PID制御での出力を制限
def output_limit(value)
    limit_val = 90
    output_val = value
    if value >= limit_val
        output_val = limit_val
    elsif value <= (-1 * limit_val)
        output_val = (-1 * limit_val)
    end
    return output_val
end
#*****************************************************************************
# ＰＩＤキャッシュの初期化
# 引数 : 無し
# 返り値 : 無し
#*****************************************************************************
def pidcash_reset
    Speaker.tone(:c4, 400)
#'    $diff[0] = 0
#    $diff[1] = 0
    $integral = 0
end
#mainプログラム
begin
    a = []
    a << ["out"]
    # LCD??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽﾊ表??ｿｽ?ｿｽ??ｿｽ?ｿｽ
    LCD.puts "ev3way_sample.rb"
    LCD.puts "--- mruby version ---"
    Speaker.volume = 1
    # 各オブジェクトを生成・初期化する
  #  $sonar = UltrasonicSensor.new(SONAR_SENSOR)
    $color = ColorSensor.new(COLOR_SENSOR)
    $color.reflect
    $touch = TouchSensor.new(TOUCH_SENSOR)
    $motor_l = Motor.new(LEFT_MOTOR)
    $motor_r = Motor.new(RIGHT_MOTOR)
    $motor_t = Motor.new(TAIL_MOTOR)

     # LED:オレンジ キャリブレーション状態
    LED.color = :orange
    loop {
        # タッチセンサが押されるまで待つ
        break if $touch.pressed?
        RTOS.delay(10)
    }
    $gyro  = GyroSensor.new(GYRO_SENSOR)
        Speaker.tone(:a4, 200)
    RTOS.delay(1000)

    forward = turn = 0
    tail_plus = 0
    flag = 0
    pidchange = 0

    lap = 0
    lap_diff = 0
    lap_start = 0
    tc = 5
    lm = 1
    rm = 1
    gyro_offset = 1
    g = $gyro.rate.to_f
    i = 50
#var sakimae
    next_flag = 0
    tail = TAIL_ANGLE_DRIVE
    eeee = 0
    sp = 0
    step_point = []
    step_point_select = 0
    step_ex = 0 #free var
    range = 0
    spin_point = 0
    back_point = 0
    search_flag = 0
    obstacle_end = 0
#var gomi
    # aaaaaa = 0
    # bbbbbb = 0
    # tttttt = 0

    puts "#{Battery.mV.to_f}mV"

    # LED:??ｿｽ?ｿｽI??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽW ??ｿｽ?ｿｽL??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽu??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ[??ｿｽ?ｿｽV??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ


    # 黒・白色のキャリブレーション
    $black_value = color_calibration
    puts "black::#{$black_value}"
        RTOS.delay(1000)
    $white_value = color_calibration
    puts "white::#{$white_value}"
    c = 0
    threshold = (($black_value + $white_value) / 2).round
    puts "black:#{$black_value},white:#{$white_value}"
        RTOS.delay(1000)
    $motor_t.reset_count

    # ??ｿｽ?ｿｽX??ｿｽ?ｿｽ^??ｿｽ?ｿｽ[??ｿｽ?ｿｽg??ｿｽ?ｿｽﾒ機
    LCD.puts "Ready to start"
    loop {
        # 完全停止用角度に制御
        tail_control(TAIL_ANGLE_STAND_UP + 3)
        #TAIL微調整
        if Button[:up  ].pressed?
            TAIL_ANGLE_STAND_UP += 1
            puts "TAIL_ANGLE_STAND_UP:#{TAIL_ANGLE_STAND_UP}"
            RTOS.delay(200)
        end
        if Button[:down].pressed?
            TAIL_ANGLE_STAND_UP -= 1
            puts "TAIL_ANGLE_STAND_UP:#{TAIL_ANGLE_STAND_UP}"
            RTOS.delay(200)
        end
        eeee = $color.reflect
        if (threshold - 3 < eeee) && (threshold + 3 > eeee)
          Speaker.tone(:e4, 200)
        end
	# タッチセンサが押されるまで待つ
        break if $touch.pressed?
        RTOS.delay(10)
    }

    lap_start = RTOS.msec #start_time_save

    RTOS.delay(300) #??ｿｽ?ｿｽZmsec??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽﾉス??ｿｽ?ｿｽ^??ｿｽ?ｿｽ[??ｿｽ?ｿｽg

    # 走行モータエンコーダーリセット
    $motor_l.reset_count
    $motor_r.reset_count
    # ジャイロセンサリセット
    $gyro.reset

    # LED:緑 走行状態
    LED.color = :green


	loop {
    	ss_flg = 0
	    loop {
	        start = RTOS.msec
	        g = $gyro.rate.to_f
	        if g < 0
	        	ss_flg += 1
	        	tc+=1
	        end
	        tail_control(TAIL_ANGLE_STAND_UP + tc)
	        puts "g = #{g}"
	        puts "tc = #{tc}"
	        ss_flg = 10 if g >= 8 #65
	        break if ss_flg >= 10 #65

	        #RTOS.delay(10)
	        wait = 4 - (RTOS.msec - start)
	        RTOS.delay(wait) if wait > 0
	    	}
	        break if ss_flg >= 10 #65
	    }


    Speaker.volume = 2

		
		graph = []
		
		gmr = $motor_r.count
		gml = $motor_l.count

    loop{
        start = RTOS.msec
		tail_control(TAIL_ANGLE_DRIVE)
        turn = pid_control(threshold,$color.reflect)

        #--------------------------------------------------??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ]??ｿｽ?ｿｽﾊゑｿｽ??ｿｽ?ｿｽﾆゑｿｽPID??ｿｽ?ｿｽﾆ托ｿｽ??ｿｽ?ｿｽx
        motor_w_count = $motor_l.count + $motor_r.count

        # # pid_adjust
        # aaaaaa = turn > 0 ? 1 : -1
        # if aaaaaa != bbbbbb && pidchange == 7#( pidchange == 5 || pidchange == 6 )
        #   t = RTOS.msec - tttttt
        #   print "t--",t,"\n" if t > 20
        #   tttttt = RTOS.msec
        # end
        # bbbbbb = aaaaaa

        if iremono[pidchange][0] < motor_w_count #+ 15100
            lap_diff = RTOS.msec - lap_start - lap
            lap = RTOS.msec - lap_start
            puts "pid:#{pidchange}\t#{lap}ms\t#{lap_diff}ms"
          	Speaker.tone(:f4, 200)
            pidcash_reset()
            lm = iremono[pidchange][1]
            rm = iremono[pidchange][2]
            KP = iremono[pidchange][3]
            KI = iremono[pidchange][4]
            KD = iremono[pidchange][5]
            i  = iremono[pidchange][6]
            threshold  += iremono[pidchange][7]
            gyro_offset = iremono[pidchange][8]

            pidchange += 1
            if pidchange >= iremono.size - 1
              flag = 2
              print "pidchange:",pidchange,">>sakimae\n"
            end
        end

				#p $color.reflect

				#灰色探知
				gc = -1
				gmrc = $motor_r.count - gmr
				gmlc = $motor_l.count - gml

				
				
				if gmrc>gmlc
					gc = (gmrc - gmlc)
				elsif gmrc<gmlc
					gc = gmlc - gmrc
				else
					gc = 0
				end
				gray_arr << gc
				gray_arr2 << $color.reflect

				if gray_arr.length >= GRAY_RANGE
					mcs= (gray_arr.standard * 100).to_i
					print "mcs :#{mcs}"
					graph << mcs
					cs = (gray_arr2.standard * 100).to_i
					puts "/cs :#{cs}"
					
					ser = 0
					if cs < mcs
						ser = mcs - cs
					end
					puts "/ser:#{ser}"
					if ser > 50
						Speaker.tone(:f4, 200)
					end
					gray_arr.clear
					gray_arr2.clear
				end
				gmr = $motor_r.count
				gml = $motor_l.count
				
=begin
				if gray_arr.length >= GRAY_RANGE
					min = gray_arr[0]
					max = gray_arr[0]
					gray_arr.each do |v|
						if v > max
							max = v
						end
						if v < min
							min = v
						end
					end
					range = (max - min).to_i
					graph << range#gray_arr.standard
					p range#gray_arr.standard
					gray_arr.clear
				end
				#end
=end
        g = $gyro.rate.to_f
        pwm_l, pwm_r = Balancer.control(
        forward+i,
        turn,
        g,
        gyro_offset,
        $motor_l.count.to_f,
        $motor_r.count.to_f,
        Battery.mV.to_f)

        $motor_l.stop(true) if pwm_l == 0
        $motor_l.power = pwm_l * lm
        $motor_r.stop(true) if pwm_r == 0
        $motor_r.power = pwm_r * rm

#            a << [g,pwm_l, pwm_r,wait,$color.reflect, threshold]
        if pwm_l + pwm_r == 200 || pwm_l + pwm_r == -200
            a << [$motor_l.count,$motor_r.count] if c == 0
            c=c+1
            break if c == 250
        else
            c = 0
        end

        # 4msec??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽN??ｿｽ?ｿｽ??ｿｽ?ｿｽ
        wait = 4 - (RTOS.msec - start)
        RTOS.delay(wait) if wait > 0

    }
    $motor_l.stop(false)
    $motor_r.stop(false)
    $motor_t.stop(false)
    puts RTOS.msec - lap_start
    a.each do |b|
        puts b
    end
	p graph
rescue => e
    LCD.error_puts e
end
