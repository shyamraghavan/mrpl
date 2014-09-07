function [ sectorLaserRanges ] = inSector( laserRanges, beginningIndex, endIndex )
    
    beginningIndex = mod(beginningIndex, 360);
    endIndex = mod(endIndex, 360);
        
    if beginningIndex > endIndex
        sectorLaserRanges = laserRanges;
        sectorLaserRanges(endIndex:beginningIndex) = 10*ones(1,beginningIndex - endIndex + 1);
    else
        sectorLaserRanges = 10 * ones(1,360);
        sectorLaserRanges(beginningIndex:endIndex) = laserRanges(beginningIndex:endIndex);
    end
    
end