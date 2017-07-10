function [data skip] = readOrocosData(filenameOLD, numSpaces)
    filenameNEW = 'tmp.dat';
    
    fidOLD = fopen(filenameOLD);
    fidNEW = fopen(filenameNEW,'w');
    tlineOLD = fgets(fidOLD);
    skip = 0;
    while ischar(tlineOLD)
        tlineNEW = strrep(tlineOLD, '   ', ' ');
        tlineNEW = strrep(tlineNEW, '  ', ' ');
        l=length(strfind(tlineNEW,' '));
        if(l == numSpaces)
            fwrite(fidNEW,tlineNEW);
        else
            skip = skip+1;
        end
        tlineOLD = fgets(fidOLD);
    end
    fclose(fidOLD);
    fclose(fidNEW);
    
    data = dlmread(filenameNEW, ' ', 1, 1);
    delete(filenameNEW);
end
