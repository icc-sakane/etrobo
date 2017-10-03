    case flag
    when 0  # スタートからゴールまではこれを使用
      tail = TAIL_ANGLE_DRIVE
    when 1  # ゴールから障害物まで（使用してない）
      gyro_offset = 2
      flag = 2 if motor_w_count > 1000
    when 2  # 段差検知
      next_flag = flag + 1
      flag = 300
    when 3  # バック(back_point:バック距離)
      next_flag = flag + 1
      back_point = 100
      flag = 600
    when 4  # 段差上がる
      next_flag = flag + 1
      flag = 400
    when 5 # 段差検知
      next_flag = flag + 1
      flag = 300
    when 6 # バック
      back_point = 100
      next_flag = flag + 1
      flag = 600
    when 7 # 段差上がる
      next_flag = flag + 1
      flag = 400
    when 8 # 回転
      $motor_t.rotate(0,0,false)
      $motor_l.rotate(0,0,true)
      $motor_r.rotate(360,6,true)
      RTOS.delay(2000)
      flag = flag + 1
    when 9 # 段差降りる
      next_flag = flag + 1
      flag = 1000
    when 10 # 車庫まで走る
      next_flag = flag + 1
      flag = 1100
    when 11 # テイル下す
      next_flag = flag + 1
      flag = 100
    when 12　#　待ち
      RTOS.delay(10000)
    when 100 # flag 100 : tail drive start : tail overwrite
      #tailは値が大きいほど下に下がる
      #tailにテイル走行用の値を入れる（おそらく以下のプログラムで完全に降りているはずなので不要）
      tail = TAIL_ANGLE_TAIL_DRIVE
      #ちょい前に進む
      $motor_l.rotate(110,12,false)
      $motor_r.rotate(110,12,false)
      RTOS.delay(80)
      #そこそこテイルを下げる
      $motor_t.rotate(TAIL_ANGLE_TAIL_DRIVE - 8,10,true)
      #前に進みながらゆっくり残りの８を下げる
      $motor_l.rotate(40,7,false)
      $motor_r.rotate(40,7,true)
      $motor_t.rotate(8,1,true)
      RTOS.delay(2000)
      puts "tail drive start"
      flag = next_flag

    when 300 # flag 305..306 : 従来の段差検知（モーター逆回転検知）
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
      step_ex = sp + 100
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

    when 600 #バック

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