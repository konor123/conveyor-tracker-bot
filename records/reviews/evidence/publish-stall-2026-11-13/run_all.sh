#!/bin/bash
# run_stall.sh를 설정 3종 × 시나리오 2종 × 3회 돌리고 요약을 summary.txt에 모은다(이 폴더에서 실행).
: > summary.txt
for rep in 1 2 3; do for v in default async udp; do for sc in spaced burst; do
  bash run_stall.sh $v $sc $rep >> summary.txt 2>&1
done; done; done
