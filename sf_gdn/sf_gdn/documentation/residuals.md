<ins>3D odometry residual</ins>

![transforms](pngs/3D_odometry_residual.png)  
$\ residual(_WT_A,\ _WT_B,\ _AT_B,\ \sigma)  $  
$\ _WT_A - \textup{Previous pose, assume it's correct} $  
$\ _WT_B - \textup{The transform being optimized / calculated} $  
$\ _AT_B - \textup{From measurement} $  
$\ \sigma - \textup{TODO: MAGICALLY HANDLED WITH TANGENT SPACE PERTUBATIONS} $  

$\ _A\hat{T}_B = (_WT_A)^{-1}\times _WT_B  $  
$\ tangent\_error = _A\hat{T}_B.local\_coordinates(_AT_B) $

<ins>Matching residual (2D) </ins>  

![transforms](pngs/matching_residual.png)  
$\ residual(_Wt_L,\ _WT_B,\ _Bt_L,\ \sigma) $  
$\ _Wt_L - \textup{Known global position of landmark} $  
$\ _WT_B - \textup{The transform being optimized / calculated} $  
$\ _Bt_L - \textup{Position measurement of landmark from current pose} $  
$\ \sigma - \textup{Standard deviation of measurement} $  

$\ _B\hat{t}_L = (_WT_B)^{-1}\times _Wt_L $  
$\ error = \frac{_B\hat{t}_L - _Bt_L}{\sigma} $  
Note - The bigger the stddev, the less effect on the optimization
