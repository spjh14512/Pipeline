# 처리할 것들
Branch instructinon
- --실행 전에 5사이클 stall (Z 의존성)--
- 실행 후에 5사이클 stall (registerfile의 pc값이 변한 이후 inst가 갱신된 pc값의 instruction을 fetch할 때까지 기다림)
- pc값을 registerfile의 pc 포트에 연결해놓고, IDEX register에 저장된 BEQ signal과 Z flag가 모두 1이라면 PCWrite flag를 활성화하여 pc값에 branch instruction의 결과를 저장하도록 한다.

# 질문사항
- pc가 바뀌는 경로가 무엇인지.


# 구현해야 할 것들
- branch hazard - branch할 때 stall
- stage register - 완료
- data hazard - 쓰기 후 읽기 시 stall
- structural hazard - 모든 instruction (branch 제외)를 5사이클로 통일
- control signal 뒤로 넘겨주기 (stage register 통해서)
- data hazard detection
- pc 어케됨???
- 문서화

# 6월 8일
- Stage Register 추가
- registerfile에서 input으로 r15를 추가하여 항상 값을 변경할 수 있도록.
- Branch(B/BEQ) 감지, pc값 업데이트 (stall 제외)