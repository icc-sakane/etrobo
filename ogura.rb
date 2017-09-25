#####################################################################################
#
#     Rコース用[小倉]( ；´Д｀)
#
#####################################################################################

include EV3RT_TECS

# ポート番号の設定
TOUCH_SENSOR = :port_1
COLOR_SENSOR = :port_3
GYRO_SENSOR  = :port_4
SONAR_SENSOR = :port_2

TAIL_MOTOR   = :port_a
RIGHT_MOTOR  = :port_b
LEFT_MOTOR   = :port_c

#いろいろ初期値
gyro_offset = 1
SONAR_ALERT_DISTANCE = 30
TAIL_ANGLE_STAND_UP = 91
TAIL_ANGLE_DRIVE = 3
TAIL_ANGLE_TAIL_DRIVE = 75 #tail drive
P_GAIN = 1.5
PWM_ABS_MAX = 60


# PID制御に関わる定数をセット
# 周期を設定
PERIOD = 0.004

KP = 1.7
KI = 0
KD = 0.23

iremono = [
  #0.21
# => [motor_w_count,lm,rm,KP,KI,KD,i,threshold,gyro_offset]
  [    0, 1, 1,0.126,0.162, 0.04, 100,  0,  -3], # 1streat
  [ 1000, 1, 1,0.126,0.162, 0.04, 100,  0,  -3], # 1streat
  [ 6000, 1, 1,0.996,0.176,0.044,  100, -5,  -5], # 1carb
  [ 15200, 1, 1,0.126,0.162, 0.04, 100,  0,  -5], # 2streat
  [15500, 1, 1,0.996,0.176, 0.044,  80,  -5,  -7], # 2streat
  [20000, 1, 1,  0.996,0.176,0.044,  70, -5,  -7], # 2carb_1
  [24000, 1, 1, 0.126,0.162,0.04,  100,  0,  -3], # 2carb_mid
  [29600, 1, 1, 0.996,0.176,0.044,  50, -5,  -5], # 2carb_2
  [33000, 1, 1,0.126,0.162, 0.04,  25, 0,  -5], # 3streat
 # [45000, 1, 1,0.126,0.162, 0.04, 100,  5,  -5], # 3streat
  # [15100, 1, 1,0.126,0.162, 0.04, 100,  5,  4], # test_3streat_start
  # [16000, 1, 1,0.126,0.162, 0.04, 100,  5,  8], # test_3streat
  [38000, 1, 1, 0.126, 0.17,0.042,  100,  3,  -5], # 3carb
  # [18600, 1, 1,  1.7,    1, 0.22,  50,  3,  0], # 3carb_old
  #[47000, 1, 1,0.126,0.162, 0.04,  100, -3,  -5], # 4streat_goal
  #[49000, 1, 1,0.126,0.162, 0.04,  100,  0,  -5], # 4streat
  # [22000, 1, 1,  1.7,    0,    0,  15, -3,  0], # 4streat_old
  [999999] #end
]

$diff = Array.new(2)
$integral = 0

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
  $integral += (($diff[1] || 0 ) + ($diff[0] || 0)) / 2.0 * PERIOD

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
#*****************************************************************************
# ゲート走行
# 引数 : 無し
# 返り値 : 無し
#*****************************************************************************
def gate
  left_m = 1
  right_m = 1

  lap_diff = RTOS.msec - lap_start - lap
  lap = RTOS.msec - lap_start

  KP = 0.3
  KI = 0.1
  KD = 0.02
  speed = 6

  if distance <= 20 && distance > 0 #ゲートにめっちゃ近づいたとき
    puts "in flag6"
    pidflag = 7
    puts "start_motor = #{$motor_t.count}"
    mtc = $motor_t.count
    Speaker.tone(:e4, 400)

    $motor_r.rotate(60, 10, false)
    $motor_l.rotate(60, 10, false)
    #$motor_t.rotate(58, 15, true)
    $motor_t.rotate(70, 15, true)#65-10
    RTOS.delay(1000)
    $motor_t.rotate(-6, 15, true)#2段階で尻尾を下げ
    RTOS.delay(1000)
    $motor_t.rotate(-6, 15, true)#2段階で尻尾を下げ
    RTOS.delay(1000)


    puts "end_motor = #{$motor_t.count}"

    loop{
      $motor_t.rotate( gate_tail_stand_up - $motor_t.count , 3, false)
      RTOS.delay(200)
      $motor_r.rotate( 2, 35, false)
      $motor_l.rotate( 2, 35, true)
      RTOS.delay(200)
      $motor_r.rotate( -2, 35, false)
      $motor_l.rotate( -2, 35, true)
      # puts "調整のtail_motor_count = #{$motor_t.count}"
      break if $motor_t.count == gate_tail_stand_up
    }

    RTOS.delay(1000)

    gate_motor_power_a = 1
    gate_motor_power_b = 8

    tail_flag = 0
    plus_rotate = 0
    loop{
      start = RTOS.msec
      distance = $sonar.distance

      puts "color = #{$color.reflect}"
      # puts "distance = #{distance}"
      #puts "c = #{c}"

      if $color.reflect >= tail_white - 2  #白の時と白以外の時
        $motor_l.power = gate_motor_power_a #2 -> 5 -> 2
        $motor_r.power = gate_motor_power_b #5 -> 2 -> 5
      else
        $motor_l.power = gate_motor_power_b
        $motor_r.power = gate_motor_power_a
      end


      if  distance >= 130 && tail_flag == 0 #近くに行った後、ゲートをくぐったか確認する処理 現在130
        c = c + 1
      else
        c = 0
      end

      if distance <= 50 && distance > 0 && tail_flag == 1
        Speaker.tone(:a4, 200)
        tail_flag = 0
      end

      if c == 2500 #時間決め
        $motor_l.stop
        $motor_r.stop
        RTOS.delay(1500)

        if gate_c == 2 #２回カウントしたらブレイク
          $motor_l.reset_count
          $motor_r.reset_count
          Speaker.tone(:b4, 500)
          c = 0
          break
        end

        loop{
          $motor_t.rotate( gate_tail_stand_up + 10 - $motor_t.count , 3, false)
          $motor_r.rotate( -1, 40, false)
          $motor_l.rotate( -1, 40, true)
          RTOS.delay(100)
          $motor_r.rotate( 1, 40, false)
          $motor_l.rotate( 1, 40, true)
          RTOS.delay(100)
          # puts "調整のtail_motor_count = #{$motor_t.count}"
          break if $motor_t.count == gate_tail_stand_up + 10
        }


        RTOS.delay(500)
        $motor_l.rotate(330 + plus_rotate , 15, false)#反転する処理
        $motor_r.rotate(-330 - plus_rotate , 15, true)
        RTOS.delay(500)

        plus_rotate  = 30

        loop{
          $motor_t.rotate( gate_tail_stand_up - $motor_t.count , 3, false)
          $motor_r.rotate( -1, 40, false)
          $motor_l.rotate( -1, 40, true)
          RTOS.delay(100)
          $motor_r.rotate( 1, 40, false)
          $motor_l.rotate( 1, 40, true)
          RTOS.delay(100)
          puts "調整のtail_motor_count = #{$motor_t.count}"
          break if $motor_t.count == gate_tail_stand_up
        }

        RTOS.delay(1000)

        c = 0

        tail_flag = 1
        gate_c = gate_c + 1

        gate_motor_power_a , gate_motor_power_b = gate_motor_power_b , gate_motor_power_a

        puts "tail_white = #{tail_white}"
      end


    }

    parking_count = 1100 #ゲート終了からガレージまでの距離
    stand_up_flag = 1

    loop{ #最後の駐車場に行く処理
      if $color.reflect >= tail_white - 5
        $motor_l.power = gate_motor_power_a #2 -> 5 -> 2
        $motor_r.power = gate_motor_power_b #5 -> 2 -> 5
      else
        $motor_l.power = gate_motor_power_b
        $motor_r.power = gate_motor_power_a
      end
      motor_w_count = $motor_l.count + $motor_r.count
      if motor_w_count >= parking_count
        $motor_l.stop
        $motor_r.stop
        if stand_up_flag == 1
          loop{
            $motor_t.rotate( gate_tail_stand_up + 10 - $motor_t.count , 3, false)
            $motor_r.rotate( -1, 35, false)
            $motor_l.rotate( -1, 35, true)
            RTOS.delay(100)
            $motor_r.rotate( 1, 35, false)
            $motor_l.rotate( 1, 35, true)
            RTOS.delay(100)
            puts "調整のtail_motor_count = #{$motor_t.count}"
            break if $motor_t.count == gate_tail_stand_up + 10
          }
          stand_up_flag = 0
        end
        RTOS.delay(6000)
        $motor_l.reset_count
        $motor_r.reset_count
        parking_count = 100
        break if $touch.pressed?
      end
    }

  end
end
############mainプログラム#####################################################################
begin
  a = []
  a << ["out"]
  LCD.puts "ev3way_sample.rb"
  LCD.puts "--- mruby version ---"
  Speaker.volume = 1
  # 各オブジェクトを生成・初期化する
  $sonar = UltrasonicSensor.new(SONAR_SENSOR)
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

#いろいろ初期値
  forward = turn = 0
  tail_plus = 0
  flag = 0
  pidchange = 0
#-------kobayashi----------------------
  pidflag = -1
  distance = 1
  gate_c = 0
#--------------------------------------
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

  puts "#{Battery.mV.to_f}mV"


  # 黒・白色のキャリブレーション
  $black_value = color_calibration
  puts "black::#{$black_value}"
      RTOS.delay(1000)
  $white_value = color_calibration
  puts "white::#{$white_value}"
  c = 0
  threshold = (($black_value + $white_value) / 2).round
  puts "black:#{$black_value},white:#{$white_value}"
########尻尾調整ですｗ######################################################

  $motor_t.reset_count

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
    # タッチセンサが押されるまで待つ
    break if $touch.pressed?
    RTOS.delay(10)
  }

  TAIL_ANGLE_DRIVE = TAIL_ANGLE_STAND_UP - 88
  gate_tail_stand_up = TAIL_ANGLE_STAND_UP - 22 #ゲートをくぐる角度なんじゃ
  RTOS.delay(500)
  Speaker.tone(:a4, 200)
########ゲートをくぐるときのカラー#####################################################
    #$motor_t.rotate(58, 20, true)
#くぐるときの黒
  tail_control(gate_tail_stand_up)
  tail_black = color_calibration
  puts "tail_black = #{tail_black}"
  Speaker.tone(:a4, 200)
  RTOS.delay(500)
#くぐるときの白
  tail_white = color_calibration
  puts "tail_white = #{tail_white}"
  Speaker.tone(:a4, 200)
  RTOS.delay(500)

  loop {
    tail_control(TAIL_ANGLE_DRIVE)
    # タッチセンサが押されるまで待つ
    break if $touch.pressed?
    RTOS.delay(10)
  }
  Speaker.tone(:a4, 200)
  RTOS.delay(500)
##########スタート待機###########################################################
  LCD.puts "Ready to start"
  loop {
    # 完全停止用角度に制御

    tail_control(TAIL_ANGLE_STAND_UP)
    now_color = $color.reflect
    #いい感じのとこに置いたら「びーびー」鳴る
    if (threshold - 3 < now_color) && (threshold + 3 > now_color)
      Speaker.tone(:e4, 200)
    end
    # タッチセンサが押されるまで待つ
    break if $touch.pressed?
    RTOS.delay(10)
  }

  lap_start = RTOS.msec #start_time_save

  RTOS.delay(300)

  # 走行モータエンコーダーリセット
  $motor_l.reset_count
  $motor_r.reset_count
  # ジャイロセンサリセット
  $gyro.reset

  # LED:緑 走行状態
  LED.color = :green

  loop {
    ss_flg = 0
    #ss_flgが10以上(角速度がいい感じ)になるまでひたすらループ
    loop {
      start = RTOS.msec
      #g=角速度
      g = $gyro.rate.to_f
      #後ろに倒れてたら？
      if g < 0
        ss_flg += 1
        tc+=1
      end
      #ている上げるぜ
      tail_control(TAIL_ANGLE_STAND_UP + tc)
      puts "g = #{g}"
      puts "tc = #{tc}"
      ss_flg = 10 if g >= 8 #65
      break if ss_flg >= 10 #65

      wait = 4 - (RTOS.msec - start)
      RTOS.delay(wait) if wait > 0
    }
    break if ss_flg >= 10 #65
  }
  Speaker.volume = 20
##########走行メイン#########################################################
  loop{
    start = RTOS.msec
    # ライントレース
    turn = pid_control($color.reflect,threshold)

    motor_w_count = $motor_l.count + $motor_r.count
    #モーター回転数に対応する値を入れてく
    if iremono[pidchange][0] < motor_w_count #+ 15100
      lap_diff = RTOS.msec - lap_start - lap
      lap = RTOS.msec - lap_start
      puts "pid:#{pidchange}\t#{lap}ms\t#{lap_diff}ms"
      Speaker.tone(:e5, 200)
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
      #ここで段差プログラムに入るんじゃない??
      if pidchange >= iremono.size - 1
        flag = 2
        print "pidchange:",pidchange,">>sakimae\n"
      end
    end

######なんかいっぱい書いてある…これって段差プログラムじゃね？ww################################
    #スタートからゴールまでは0しか使ってない( *´艸｀)
    #flag:状態
    #next_flag:小さい数字（1～２５かな？）をflagに入れるための入れ物
    #正直100以上は関数にすればいいのに…
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
    when 100 # flag 100 : tail drive start : tail overwrite
      tail = TAIL_ANGLE_TAIL_DRIVE
      $motor_l.rotate(110,12,false)
      $motor_r.rotate(110,12,false)
      RTOS.delay(80)
      $motor_t.rotate(TAIL_ANGLE_TAIL_DRIVE - 8,10,true)
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
      i = 3
      lap_diff = RTOS.msec - lap_start - lap
      flag = next_flag if lap_diff >= 1000
#秒速用---とりあえずグラフから
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
      per_mseconds = []
      step_serch_time = RTOS.msec
    when 301
      lap_diff = RTOS.msec - lap_start - lap
      if lap_diff > 200
        per_mseconds << (now_m_count - motor_w_count) / (step_serch_time - RTOS.msec)
        lap = RTOS.msec - lap_start
        #if sp - 30 > motor_w_count && search_flag == 1
        #  pidcash_reset()
        #  flag = next_flag
        #  i = 0
        #  step_point << sp
        #end
        #search_flag = 1 if sp + 30 < motor_w_count && search_flag == 0
        #sp = motor_w_count
      end
      now_m_count = motor_w_count
      step_serch_time = RTOS.msec
    #lap_start:スタート
    #start:毎回のループの開始
    #lap:スタートしてから段差プログラムに入るまでかかった時間
    #lap_diff:段差検知プログラム実行時間
    #sp:開始前のモーター回転数
    #search_flag:検知開始フラグ
    when 305 # flag 305..306 : search step_2 : not tail overwrite
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
    when 306
      lap_diff = RTOS.msec - lap_start - lap#段差検知してる時間何秒？
      if lap_diff > 200#0.2秒探した？
        lap = RTOS.msec - lap_start
        #モーターがあれから-30＆検知開始してる？
        if sp - 30 > motor_w_count && search_flag == 1
          pidcash_reset()
          flag = next_flag
          i = 0
          step_point << sp
        end
        #モーターが開始から３０以上なら検知開始
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
#############################################################################

    distance = $sonar.distance #kobayashi
    #ているを元の高さに
    tail_control(tail)

#------------------------------ここからゲート--------------------------------
    #if pidflag == 6 && distance <= 25 && distance > 0 #ゲートに近づいたとき
    gate() if iremono[pidchange][8] < motor_w_count #kobayashi

#-----------------------------------------------------------------------

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

    if pwm_l + pwm_r == 200 || pwm_l + pwm_r == -200
      a << [$motor_l.count,$motor_r.count] if c == 0
      c = c+1
      break if c == 250
    else
      c = 0
    end

    wait = 4 - (RTOS.msec - start)
    RTOS.delay(wait) if wait > 0

  }
  $motor_l.stop(false)
  $motor_r.stop(false)
  $motor_t.stop(false)
  puts RTOS.msec - lap_start
  puts per_mseconds
  a.each do |b|
      puts b
  end
  rescue => e
  LCD.error_puts e
end
