data.raw= radio_output_1_procesado;

data.raw['lat_m']<-NULL
data.raw['lon_m']<-NULL

data.raw[is.na(rowSums(data.raw)),0]

data.raw = na.omit(data.raw)
data.info <- data.frame(
      mean = sapply(data.raw,mean),
       sd = sapply(data.raw,sd),
       var = sapply(data.raw,var)
)
