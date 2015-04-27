
function velocity_b = velocity_t_to_velocity_b(velocity_t,attitude_t)
N = length(velocity_t);
velocity_b = zeros(3,N);
for k=1:N
   Cbt = FCbn(attitude_t(:,k)); 
   Ctb = Cbt' ;
   velocity_b(:,k) = Ctb* velocity_t(:,k) ;
   vt=velocity_t(:,k) ;
   vb = Ctb* velocity_t(:,k) ;
end
