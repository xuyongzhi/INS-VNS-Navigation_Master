N = 4000;
step=400;
im = ones(N);
featureNum = fix(N/step) ;

width = fix(unifrnd(40,150,1,featureNum*featureNum));
i=0;
for k=1:featureNum
    
    for m=1:featureNum
        i=i+1;
        x0=step*m-step/2;
        y0=step*k-step/2;
        nn=width(i);
        im(x0-nn:x0+nn,y0-nn:y0+nn)=0;
        mm=15;
        im(x0-nn+mm:x0+nn-mm,y0-nn+mm:y0+nn-mm)=1;
    end
    
 
end


imwrite(im,'res\L1.bmp');
imwrite(im,'res\L2.bmp');
imwrite(im,'res\R1.bmp');
imwrite(im,'res\R2.bmp');
disp('dotTieTu ok')



