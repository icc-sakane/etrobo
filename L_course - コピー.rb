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

#??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽL??ｿｽ?ｿｽﾌ定数??ｿｽ?ｿｽl??ｿｽ?ｿｽﾍ個托ｿｽ/??ｿｽ?ｿｽﾂ具ｿｽ??ｿｽ?ｿｽﾉ搾ｿｽ??ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ??ｿｽ?ｿｽﾄ変更??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽK??ｿｽ?ｿｽv??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽﾜゑｿｽ
gyro_offset = 1		  #ロボット(固有名)のジャイロセンサー固有基礎設定値
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
<<<<<<< HEAD:L_course - コピー.rb
    [    0, 1, 1,0.126,0.162, 0.04, 100,  0, 5], # pid0
    [ 1000, 1, 1,0.106,0.162, 0.04, 100,  3, 6], # pid1(1st_straight)
    [ 6000, 1, 1,0.696,0.186,0.034,  100, 1,  2], # pid2(1carb)
    [ 10000, 1, 1,0.696,0.186,0.034,  100, 1,  2], # pid3(1carb)
    [ 12000, 1, 1,0.106,0.182, 0.04, 100,  4,  3], # pid4(2st_straight)
    [14600, 1, 1, 0.596,0.225,0.056,  100,  3, 3], # pid5(2carb_mid)
    [19000, 1, 1, 0.596,0.196,0.044,  100, 1,  2], # pid6(2carb_2)
		[21000, 1.1, 1, 0.896,0.16,0.044,  100, -6, 2], #pid7
    [23500, 1, 1,0.2,0.162, 0.034,  100,  0, 5], #pid8(3st_straight)
    [29000, 1, 1,0.2,0.162, 0.034, 60,  0,  5], # pid9(3carb)
=======
    [    0, 1, 1,0.126,0.162, 0.04, 100,  0, 0], # pid0
    [ 1000, 1, 1,0.106,0.162, 0.04, 100,  3, 1], # pid1(1st_straight)
    [ 6000, 1, 1,0.696,0.186,0.034,  100, 1,  -3], # pid2(1carb)
    [ 10000, 1, 1,0.696,0.186,0.034,  100, 1,  -3], # pid3(1carb)
    [ 12000, 1, 1,0.106,0.182, 0.04, 100,  4,  -2], # pid4(2st_straight)
    [14600, 1, 1, 0.596,0.225,0.056,  100,  3, -2], # pid5(2carb_mid)
    [19000, 1, 1, 0.596,0.196,0.044,  100, 1,  -3], # pid6(2carb_2)
	[21000, 1.1, 1, 0.896,0.16,0.044,  100, -6, -3], #pid7
    [23500, 1, 1,0.2,0.162, 0.034,  100,  0, 0], #pid8(3st_straight)
    [29000, 1, 1,0.2,0.162, 0.034, 60,  0,  0], # pid9(3carb)
	[31000, 1, 1,0.126,0.162, 0.04, 40,  0, -2], #pid10(dansa)
>>>>>>> 060ae1637f2d281106f4bae623fbf17be224c020:L_course.rb
   # [35000, 1, 1, 0.996, 0.17,0.042,  60,  3,  -5], # pid10(3carb)
   # [40000, 1, 1,0.126,0.162, 0.04,  50, -3,  -5], # pid11(4st_straight_goal)
   # [99999, 1, 1,0.126,0.162, 0.04,  50,  -3,  -5], # 4st_straight
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
    $diff[0] = 0
    $diff[1] = 0
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
        start = RTOS.msec
        g = $gyro.rate.to_f
        tail_control(TAIL_ANGLE_STAND_UP + tc)
        #puts "g = #{g}"
        # puts "tc = #{tc}"
        break if g >= 1   #65
        #RTOS.delay(10)
        wait = 4 - (RTOS.msec - start)
        RTOS.delay(wait) if wait > 0
    }


    Speaker.volume = 2
	d_cnt = 0
	flag_cnt = 0
	d_flag = 0
    loop{
        start = RTOS.msec
        # ??ｿｽ?ｿｽo??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽX??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽs??ｿｽ?ｿｽp??ｿｽ?ｿｽp??ｿｽ?ｿｽx??ｿｽ?ｿｽﾉ撰ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ
        # ライントレース
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
              #print "pidchange:",pidchange,">>sakimae\n"
            end
        end

        case flag
        when 0  # start --> center goal
          tail = TAIL_ANGLE_DRIVE
        when 1  # center goal --> obstacle
          gyro_offset = 2
          flag = 2 if motor_w_count > 1000
        when 2  # call : search step
          next_flag = flag + 1
          flag = 300
        when 3  # call : back :argment back_point
          next_flag = flag + 1
          back_point = 300
          flag = 600
        when 4  # call : go up step
          next_flag = flag + 1
          flag = 400
        when 5  # call : control traveling
          next_flag = flag + 1
          flag = 500
        when 6 # call : search step
          next_flag = flag + 1
          flag = 300
        when 7 # call : back
          back_point = 300
          next_flag = flag + 1
          flag = 600
        when 8 # call : 2 sec stop
          next_flag = flag + 1
          flag = 200
        when 9 # call : tail drive start
          next_flag = flag + 1
          flag = 100
        when 10 # call : tail drive back
          next_flag = flag + 1
          flag = 800
          step_point_select = 1
        when 11 # call : spin
          next_flag = flag + 1
          flag = 700
        when 12 # call : start
          next_flag = flag + 1
          flag = 900
        when 13  # call : control traveling
          step_point_select = 0
          next_flag = flag + 1
          flag = 500
        when 14 # call : search step
          next_flag = flag + 1
          flag = 300
        when 15 #call : back
          next_flag = flag + 1
          flag = 600
          back_point = 300
        when 16 # call : go up step
          next_flag = flag + 1
          flag = 400
        when 17 # call : control traveling
          step_point_select = 2
          next_flag = flag + 1
          flag = 500
        when 18 # call : 2 sec stop
          next_flag = flag + 1
          flag = 200
        when 19 # call : tail drive start
          next_flag = flag + 1
          flag = 100
        when 20 # call : tail drive back
          next_flag = flag + 1
          flag = 800
        when 21 # call : spin
          next_flag = flag + 1
          flag = 700
        when 22 # call : go down step
          next_flag = flag + 1
          flag = 1000
        when 23 # call : winning run
          next_flag = flag + 1
          flag = 1100
        when 24 # call : tail drive start
          next_flag = flag + 1
          flag = 100
        when 25
          RTOS.delay(10000)


#---------------------段差ココ！----------------------------------------

        g = $gyro.rate.to_f
        #100回ループ中にg+-50行かなかったらスタート
        if d_flag==0 && (g >= -50 && g <= 50)
         	d_cnt += 1
        elsif d_flag==0
        	d_cnt = 0
        end
				if d_flag==0 && (d_cnt >= 400) #スタート
        			d_flag = 1
							puts "aaaa aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
        	    Speaker.tone(:c5, 200)
        	    d_cnt = 0
				end
        if d_flag==1 && (g < -80 || g > 80)	#段差検知
        	d_flag = 2
						puts "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
					  flag_cnt = 0
						#gyro_offset = 5
        	  Speaker.tone(:d5, 200)
        	  	flag_cnt = $motor_l.count + $motor_r.count
        end
        if d_flag == 2	#段差乗り越えカウント
						if motor_w_count > flag_cnt + 1500
							i = 30
        	  Speaker.tone(:d5, 200)
						elsif motor_w_count > flag_cnt + 1400
        	  	Speaker.tone(:f4, 200)
							turn = -60
							i = 10
						elsif motor_w_count > flag_cnt + 800
        	  	Speaker.tone(:c5, 200)
							i = 10

						else
	        	  Speaker.tone(:d4, 200)
						end
        end

	        pwm_l, pwm_r = Balancer.control(
	        i,
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

        if pwm_l + pwm_r == 200 || pwm_l + pwm_r == -200
            a << [$motor_l.count,$motor_r.count] if c == 0
            c=c+1
            break if c == 250
        else
            c = 0
        end

#-----------------------------------------------------------------------


        when 100 # flag 100 : tail drive start : tail overwrite
          tail = TAIL_ANGLE_TAIL_DRIVE
          $motor_l.rotate(110,12,false)
          $motor_r.rotate(110,12,false)
          RTOS.delay(80)
          $motor_t.rotate(TAIL_ANGLE_TAIL_DRIVE - 8,15,true)
          $motor_l.rotate(40,7,false)
          $motor_r.rotate(40,7,true)
          $motor_t.rotate(8,1,true)

          RTOS.delay(2000)

          puts "tail drive start"
          flag = next_flag

        when 200  # flag 200..201 : 2 sec stop : not tail overwrite
          lap = RTOS.msec - lap_start
          flag = flag + 1
          puts "2 sec stop ..."
        when 201
          i = 2
          lap_diff = RTOS.msec - lap_start - lap
          flag = next_flag if lap_diff >= 1000

        when 300 # flag 300..301 : search step : not tail overwrite
          gyro_offset = 3
          KP = 0.8
          KI = 0.2
          KD = 0.1
          i = 12
          lap = RTOS.msec - lap_start
          flag = flag + 1
          puts "search step sequence"
          sp = motor_w_count
          search_flag = 0
        when 301
          lap_diff = RTOS.msec - lap_start - lap
          if lap_diff > 200
            lap = RTOS.msec - lap_start
            if sp - 30 > motor_w_count && search_flag == 1
              pidcash_reset()
              flag = next_flag
              i = 0
              step_point << sp
            end
            search_flag = 1 if sp + 30 < motor_w_count && search_flag == 0
            sp = motor_w_count
          end

        when 400 # flag 400..401 : go up step  : not tail overwrite
          gyro_offset = 3
          step_ex = sp + 180
          flag = flag + 1
          i = 50
          puts "go up step sequence"
        when 401
          if step_ex < motor_w_count
            flag = next_flag
            pidcash_reset()
            i = 0
            gyro_offset = 0
          end

        when 500 #flag 500..501 : control traveling : not tail overwrite
          flag = flag + 1
          range = 60
          spin_point = step_point[step_point_select] + 500
          puts "control traveling sequence"
          lap = RTOS.msec - lap_start
        when 501
          lap_diff = RTOS.msec - lap_start - lap
          i = 2
          gyro_offset = 0

          if motor_w_count > spin_point + ( range / 2 )
            i = 0
            gyro_offset = -2
            turn = 0
          elsif motor_w_count < spin_point - ( range / 2 )
            i = 6
            gyro_offset = 2
          end
          if lap_diff > 6000
            flag = next_flag
            pidcash_reset()
          end
        when 600 #flag 600..601 : back : not tail overwrite

          flag = flag + 1
          gyro_offset = -4
          puts "back sequence"
        when 601
          turn = 0
          if motor_w_count < sp - back_point
            pidcash_reset()
            flag = next_flag
            gyro_offset = 1
          end
        when 700 #flag 700 : spin : not tail overwrite
          $motor_t.rotate(0,0,false)
          $motor_r.rotate(720,6,false)
          $motor_l.rotate(-720,6,true)
          RTOS.delay(4000)
          flag = next_flag

        when 800 #flag 800 : tail drive back : not tail overwrite
          mov = -20
          mov = -100 if motor_w_count > step_point[step_point_select] - 200
          $motor_t.rotate(0,0,false)
          $motor_r.rotate(mov,6,false)
          $motor_l.rotate(mov,6,true)
          RTOS.delay(2000)
          flag = next_flag
        when 900 #flag 900 : start : tail overwrite
          tc = 0
          cnt = 0
          $motor_l.power = 30
          $motor_r.power = 30
          loop{
            g = $gyro.rate
            if g < -10
              $motor_l.stop
              $motor_r.stop
            end
            tail_control(80+tc)
            tc += 1 if cnt % 200 == 0
            cnt += 1
            puts "g : #{g}"
            break if tc > 12 + ( TAIL_ANGLE_STAND_UP - 91 )&& g > 20 #30
            wait = 4 - (RTOS.msec - start)
            RTOS.delay(wait) if wait > 0
          }
          gyro_offset = 0
          flag = next_flag
          tail = TAIL_ANGLE_DRIVE
        when 1000 #flag 1000 : go down step : tail overwrite
          $motor_t.rotate(-5,1,true)
          flag = flag + 1
          tail = (TAIL_ANGLE_TAIL_DRIVE - 10)
        when 1001
          $motor_t.rotate(0,0,false)
          loop{
            start = RTOS.msec
            eeee = $color.reflect
            if eeee < 4
              $motor_l.power = 7
              $motor_r.power = 4
            elsif eeee > 7
              $motor_l.power = 4
              $motor_r.power = 7
            else
              $motor_l.power = 7
              $motor_r.power = 7
            end
            g = $gyro.rate
            break if g >= 50
            wait = 4 - (RTOS.msec - start)
            RTOS.delay(wait) if wait > 0
          }
          tc = 0
          cnt = 0
          $motor_l.power = 30
          $motor_r.power = 30
          loop{
            start = RTOS.msec
            g = $gyro.rate
            if g < -10
              $motor_l.stop
              $motor_r.stop
            end
            tail_control(40+tc)
            tc += 2 if cnt % 100 == 0
            cnt += 1
            break if tc > 20 + ( TAIL_ANGLE_STAND_UP - 91 )&& g > 30 #30
            wait = 4 - (RTOS.msec - start)
            RTOS.delay(wait) if wait > 0
          }
          obstacle_end = motor_w_count
          gyro_offset = 4
          flag = next_flag
          tail = TAIL_ANGLE_DRIVE
        when 1100 # flag 1100 : winning run
          KP = 0.996
          KI = 0.139
          KD = 0.034
          i = 20
          gyro_offset = 2
          flag = flag + 1
        when 1101
          flag = next_flag if motor_w_count - obstacle_end > 2700
        end


        tail_control(tail)

        # ??ｿｽ?ｿｽ|??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽU??ｿｽ?ｿｽq??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽAPI??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽﾄび出??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽA??ｿｽ?ｿｽ|??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽs??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ驍ｽ??ｿｽ?ｿｽﾟゑｿｽ
        # ??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽE??ｿｽ?ｿｽ??ｿｽ?ｿｽ??ｿｽ?ｿｽ[??ｿｽ?ｿｽ^??ｿｽ?ｿｽo??ｿｽ?ｿｽﾍ値??ｿｽ?ｿｽ??ｿｽ?ｿｽ?ｿｽ */
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
rescue => e
    LCD.error_puts e
end
